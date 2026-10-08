#!/usr/bin/env python3
"""MedShelf manipulation with MoveIt 2 (MoveItPy). Drop-in replacement for mock_manipulation.

Listens:   /medshelf/pick_target          geometry_msgs/PoseStamped  (tray CENTER, Gazebo world coords)
Publishes: /medshelf/manipulation_status  std_msgs/String  EXECUTING | SUCCEEDED | FAILED
           /medshelf/manipulation_detail  std_msgs/String  which step failed and why

Per pick:
  1. Load table + shelf + the 9 cubby trays into MoveIt's planning scene (/collision_object)
  2. Open gripper, plan (OMPL) to a pre-grasp pose in front of the cubby
  3. Remove the target tray from the scene, straight-line (Pilz LIN) in to the handle
  4. Close gripper; joint feedback check (fully closed = missed the handle)
  5. Attach the tray to the gripper, lift slightly, LIN straight back out of the cubby
  6. Plan to the drop spot on the table, lower, open, detach, return home

Grasp: the tray's front wall has a 100 x 20 mm handle slot (130-150 mm above the tray bottom)
with a 25 mm bar above it. Approach horizontally; fingers close vertically around that bar.

Coordinates: Gazebo world -> robot frame by subtracting the spawn position (base_x/y/z).

Holding the tray uses the DetachableJoint "magnet" from grasp_attach.xacro / grasp.launch.py:
after the fingers close, the node reads the real tray poses from Gazebo, finds the tray whose
handle is actually between the fingers, and only attaches it if the handle is within
attach_tolerance of the gripper. That distance is reported as grasp_err (a precision metric).
"""
import math
import queue
import re
import subprocess
import struct
import threading
import time
import random
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, ParallelGripperCommand
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_GROUP = "ur_manipulator"
TOOL_LINK = "tool0"
GRIPPER_JOINT = "robotiq_85_left_knuckle_joint"
GRIPPER_OPEN, GRIPPER_CLOSED = 0.0, 0.7929
ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
HOME = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
FINGER_TIPS = ("robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link")

# Cubby tray centers in Gazebo world coords (must match trial_runner.SLOT_XYZ + 0.0875)
SLOT_CENTERS = {
    f"{lvl}_{col}": (x, -0.75, zb + 0.0875)
    for lvl, zb in (("top", 1.579), ("middle", 1.304), ("bottom", 1.029))
    for col, x in (("left", 0.25), ("center", 0.0), ("right", -0.25))
}
TRAY_SIZE = (0.175, 0.25, 0.175)
HANDLE_FWD = 0.12    # front wall center, from tray center toward the table (+y, tray yaw = pi)
HANDLE_UP = 0.075    # handle bar center above tray center (162.5 mm above bottom)


# ---------------- small math helpers ----------------
def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def apply(m, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def mat_to_quat(m):
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        return (0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s)
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        return ((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s)
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
    return ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s)


def load_stl(path, scale, rot, trans):
    """Binary STL -> shape_msgs/Mesh, with vertices already transformed into the robot frame."""
    data = Path(path).read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    mesh, index = Mesh(), {}
    for i in range(n):
        tri = MeshTriangle()
        ids = []
        for j in range(3):
            o = 84 + i * 50 + 12 + j * 12
            v = struct.unpack("<3f", data[o:o + 12])
            key = tuple(round(c, 4) for c in v)
            if key not in index:
                w = apply(rot, [c * scale for c in v])
                index[key] = len(mesh.vertices)
                mesh.vertices.append(Point(x=w[0] + trans[0], y=w[1] + trans[1], z=w[2] + trans[2]))
            ids.append(index[key])
        tri.vertex_indices = ids
        mesh.triangles.append(tri)
    return mesh


class MedShelfManipulation(Node):
    def __init__(self):
        super().__init__("medshelf_manipulation")
        p = self.declare_parameter
        p("base_x", 0.0); p("base_y", 0.12); p("base_z", 1.05)   # Gazebo spawn position of the arm
        p("tcp_offset", 0.185)       # tool0 -> center of the finger pads (m). Tune in RViz.
        p("grasp_roll_deg", 0.0)     # rotate gripper about its approach axis if fingers close sideways
        p("pregrasp_dist", 0.12)     # stop this far in front of the handle before going in
        p("retreat_dist", 0.32)      # pull straight out this far (tray is 0.25 deep)
        p("place_xyz", [0.15, -0.35, 1.12])   # tray CENTER x,y and table-top z for the drop, Gazebo coords
        p("table_top_z", 1.04)       # top of the table collision box (just under the arm base)
        p("missed_grasp_threshold", 0.75)   # knuckle angle; closed on nothing ~0.79, on the 25 mm bar ~0.56
        p("attach_tolerance", 0.08)         # max gripper-to-handle distance (m) for the magnet to grab
        p("world_name", "medshelf")
        p("pregrasp_gripper", 0.40)         # partly open (~40 mm) so the lower finger fits the 20 mm handle slot
        p("grasp_z_offset", 0.0)            # shift the grasp up(+)/down(-) if the fingers sit too high/low
        p("require_finger_contact", False)  # True = also fail if the fingers close all the way (no pinch)
        p("gripper_timeout", 25.0)          # s; Gazebo can run slower than real time
        p("lift_height", 0.03)              # raise the tray this much before pulling it out (was 0.01)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.base = (g("base_x"), g("base_y"), g("base_z"))

        self.status_pub = self.create_publisher(String, "/medshelf/manipulation_status", 10)
        self.detail_pub = self.create_publisher(String, "/medshelf/manipulation_detail", 10)
        self.co_pub = self.create_publisher(CollisionObject, "/collision_object", 10)
        self.aco_pub = self.create_publisher(AttachedCollisionObject, "/attached_collision_object", 10)
        self.create_subscription(PoseStamped, "/medshelf/pick_target", self.on_target, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.gripper = ActionClient(self, ParallelGripperCommand, "/gripper_controller/gripper_cmd")
        self.arm_direct = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.arm_pos = {}
        self.pregrasp_joints = None
        self.last_closure = float("nan")

        self.gripper_pos = None
        self.jobs = queue.Queue()
        self.held_bin = None
        self.attach_pubs, self.detach_pubs, self.attach_state = {}, {}, {}
        for b in ("gauze", "gloves", "syringes", "masks", "tape", "wipes",
                  "dressings", "saline", "specimen_cups"):
            self.attach_pubs[b] = self.create_publisher(Empty, f"/medshelf/attach/tray_{b}", 10)
            self.detach_pubs[b] = self.create_publisher(Empty, f"/medshelf/detach/tray_{b}", 10)
            self.create_subscription(String, f"/medshelf/attach_state/tray_{b}",
                                     lambda m, b=b: self.attach_state.__setitem__(b, m.data), 10)

        self.get_logger().info("Starting MoveItPy...")
        self.moveit = MoveItPy(node_name="moveit_py")
        self.arm = self.moveit.get_planning_component(ARM_GROUP)
        self.free_params = PlanRequestParameters(self.moveit, "ompl_rrtc")
        self.lin_params = PlanRequestParameters(self.moveit, "pilz_lin")
        self.ptp_params = PlanRequestParameters(self.moveit, "pilz_ptp")
        self.static_scene = self.build_static_scene()

        threading.Thread(target=self.worker, daemon=True).start()
        self.publish_status("IDLE")
        self.get_logger().info("MedShelf manipulation ready, waiting for /medshelf/pick_target")

    # ---------------- frames ----------------
    def to_robot(self, gz_xyz):
        return [gz_xyz[i] - self.base[i] for i in range(3)]

    def tool_pose(self, gz_point):
        """tool0 pose so the finger pads sit at gz_point, approaching along -y with fingers closing vertically."""
        tcp = self.get_parameter("tcp_offset").value
        roll = math.radians(self.get_parameter("grasp_roll_deg").value)
        # tool z -> world -y (into the shelf); tool x -> world +z; tool y = z cross x -> world -x
        base_rot = [[0, -1, 0], [0, 0, -1], [1, 0, 0]]
        rot = matmul(base_rot, rpy_matrix(0, 0, roll))
        x, y, z = self.to_robot(gz_point)
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y + tcp, z
        q = mat_to_quat(rot)
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q
        return ps

    # ---------------- planning scene ----------------
    def build_static_scene(self):
        share = Path(get_package_share_directory("my_arm_bringup")) / "meshes"
        objs = []

        # Shelf mesh: world model pose (0,-0.8125,0) + visual pose (-0.425,0.2125,0, roll 90deg)
        rot = rpy_matrix(math.pi / 2, 0, 0)
        trans = self.to_robot([-0.425, -0.8125 + 0.2125, 0.0])
        shelf = CollisionObject()
        shelf.header.frame_id = "world"
        shelf.id = "shelf"
        shelf.meshes.append(load_stl(share / "shelf.stl", 0.001, rot, trans))
        shelf.mesh_poses.append(Pose(orientation=Pose().orientation))
        shelf.mesh_poses[0].orientation.w = 1.0
        shelf.operation = CollisionObject.ADD
        objs.append(shelf)

        # Table as a box whose top sits just below the arm base (the arm stands on it).
        top = self.get_parameter("table_top_z").value - self.base[2]
        table = CollisionObject()
        table.header.frame_id = "world"
        table.id = "table"
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[1.0, 1.0, 0.3])
        pose = Pose()
        cx, cy, _ = self.to_robot([0.0, 0.0, 0.0])
        pose.position.x, pose.position.y, pose.position.z = cx, cy, top - 0.15
        pose.orientation.w = 1.0
        table.primitives.append(box)
        table.primitive_poses.append(pose)
        table.operation = CollisionObject.ADD
        objs.append(table)

        # Raised board on the table top. It's part of table.stl, so the flat box above misses it.
        board = self.board_box(share / "table.stl")
        if board is not None:
            objs.append(board)
        return objs

    def board_box(self, stl_path):
        """Bounding box of everything in table.stl that sits above the table surface (the raised board)."""
        data = Path(stl_path).read_bytes()
        n = struct.unpack("<I", data[80:84])[0]
        rot = rpy_matrix(-math.pi / 2, 0, math.pi)   # table model yaw pi, visual roll -pi/2
        trans = (0.5, 0.5, 1.1)                      # Rz(pi) applied to visual xyz (-0.5, -0.5, 1.1)
        surface = self.base[2]                       # arm stands on the table surface (1.05)
        pts = []
        for i in range(n):
            for j in range(3):
                o = 84 + i * 50 + 12 + j * 12
                v = struct.unpack("<3f", data[o:o + 12])
                w = apply(rot, [c * 0.001 for c in v])
                w = [w[k] + trans[k] for k in range(3)]
                if w[2] > surface + 0.015:
                    pts.append(w)
        if not pts:
            self.get_logger().warn("board_box: nothing above the table surface in table.stl")
            return None
        pad = 0.01
        x0, x1 = min(p[0] for p in pts) - pad, max(p[0] for p in pts) + pad
        y0, y1 = min(p[1] for p in pts) - pad, max(p[1] for p in pts) + pad
        z0, z1 = surface, max(p[2] for p in pts) + pad
        self.get_logger().info(f"board box (Gazebo coords): x[{x0:.3f},{x1:.3f}] "
                               f"y[{y0:.3f},{y1:.3f}] z[{z0:.3f},{z1:.3f}]")
        co = CollisionObject()
        co.header.frame_id = "world"
        co.id = "board"
        co.operation = CollisionObject.ADD
        co.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX,
                                            dimensions=[x1 - x0, y1 - y0, z1 - z0]))
        cx, cy, cz = self.to_robot([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = cx, cy, cz
        pose.orientation.w = 1.0
        co.primitive_poses.append(pose)
        return co

    def tray_object(self, slot, op=CollisionObject.ADD):
        co = CollisionObject()
        co.header.frame_id = "world"
        co.id = f"slot_{slot}"
        co.operation = op
        if op == CollisionObject.ADD:
            x, y, z = self.to_robot(SLOT_CENTERS[slot])
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            pose.orientation.w = 1.0
            co.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(TRAY_SIZE)))
            co.primitive_poses.append(pose)
        return co

    def reset_scene(self):
        # Drop anything still attached from a previous trial
        aco = AttachedCollisionObject()
        aco.link_name = TOOL_LINK
        aco.object.id = "held_tray"
        aco.object.operation = CollisionObject.REMOVE
        self.aco_pub.publish(aco)
        rm = CollisionObject()
        rm.id = "held_tray"
        rm.header.frame_id = "world"
        rm.operation = CollisionObject.REMOVE
        self.co_pub.publish(rm)
        for co in self.static_scene:
            co.header.stamp = self.get_clock().now().to_msg()
            self.co_pub.publish(co)
        for slot in SLOT_CENTERS:
            self.co_pub.publish(self.tray_object(slot))
        time.sleep(0.5)  # let both planning scenes (MoveItPy + move_group) receive it

    def attach_tray(self, slot):
        self.co_pub.publish(self.tray_object(slot, CollisionObject.REMOVE))
        aco = AttachedCollisionObject()
        aco.link_name = TOOL_LINK
        aco.object = self.tray_object(slot)
        aco.object.id = "held_tray"
        aco.touch_links = self.gripper_links()
        self.aco_pub.publish(aco)
        time.sleep(0.3)

    # ---------------- Gazebo ground truth + magnet ----------------
    def gz_tray_poses(self):
        """{bin: (x, y, z)} of every tray model, read once from Gazebo."""
        world = self.get_parameter("world_name").value
        out = subprocess.run(["gz", "topic", "-e", "-n", "1", "-t", f"/world/{world}/pose/info"],
                             capture_output=True, text=True, timeout=10).stdout
        poses = {}
        for chunk in re.split(r"\npose \{", out):
            m = re.search(r'name: "tray_([a-z_]+)"', chunk)
            pos = re.search(r"position \{([^}]*)\}", chunk)
            if not m or m.group(1) in poses:
                continue
            xyz = [0.0, 0.0, 0.0]
            if pos:
                for i, axis in enumerate("xyz"):
                    v = re.search(rf"\b{axis}: ([-\d.e+]+)", pos.group(1))
                    if v:
                        xyz[i] = float(v.group(1))
            poses[m.group(1)] = tuple(xyz)
        return poses

    def magnet_attach(self, handle_gz):
        """Attach whichever tray's handle is actually at the gripper. Returns (bin, error_m)."""
        poses = self.gz_tray_poses()
        if not poses:
            raise RuntimeError("could not read tray poses from Gazebo")
        # trays are yawed 180 deg, so the handle is +y / +z from the tray model origin (bottom center)
        handles = {b: (p[0], p[1] + HANDLE_FWD, p[2] + 0.0875 + HANDLE_UP) for b, p in poses.items()}
        b = min(handles, key=lambda k: math.dist(handles[k], handle_gz))
        err = math.dist(handles[b], handle_gz)
        if err > self.get_parameter("attach_tolerance").value:
            d = [100 * (p - h) for p, h in zip(handle_gz, handles[b])]
            raise RuntimeError(f"no handle between fingers (nearest {b}, {100 * err:.1f} cm off; "
                               f"fingers minus handle dx={d[0]:+.1f} dy={d[1]:+.1f} dz={d[2]:+.1f} cm)")
        self.attach_state.pop(b, None)
        self.attach_pubs[b].publish(Empty())
        t0 = time.monotonic()
        while self.attach_state.get(b) != "attached" and time.monotonic() - t0 < 2.0:
            time.sleep(0.05)
        self.held_bin = b
        return b, err

    def magnet_release(self):
        if self.held_bin:
            for _ in range(3):
                self.detach_pubs[self.held_bin].publish(Empty())
                time.sleep(0.05)
            self.held_bin = None

    def finger_pads_gz(self):
        """Midpoint of the two finger-tip links, measured from TF, in Gazebo world coords."""
        pts = []
        for link in FINGER_TIPS:
            t = self.tf_buffer.lookup_transform("world", link, rclpy.time.Time(),
                                                timeout=rclpy.duration.Duration(seconds=1.0))
            v = t.transform.translation
            pts.append((v.x + self.base[0], v.y + self.base[1], v.z + self.base[2]))
        return tuple((a + b) / 2 for a, b in zip(*pts))

    def current_arm_joints(self):
        if not all(j in self.arm_pos for j in ARM_JOINTS):
            return None
        return [self.arm_pos[j] for j in ARM_JOINTS]

    def move_joints_direct(self, waypoints, seconds_each=4.0):
        """Send joint waypoints straight to arm_controller (no MoveIt). Used only for recovery."""
        if not self.arm_direct.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("arm_controller action server not available")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        for i, q in enumerate(waypoints, start=1):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q]
            t = seconds_each * i
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        fut = self.arm_direct.send_goal_async(goal)
        self.wait(fut, 5.0, "direct move (send)")
        handle = fut.result()
        if not handle.accepted:
            raise RuntimeError("direct move rejected")
        self.wait(handle.get_result_async(), seconds_each * len(waypoints) * 3 + 10, "direct move")

    def ensure_home(self):
        q = self.current_arm_joints()
        if q is not None and max(abs(a - b) for a, b in zip(q, HOME)) < 0.05:
            return
        try:
            self.move("home", label="go home")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"MoveIt home failed ({e}); moving home directly")
            self.move_joints_direct([HOME], seconds_each=6.0)

    def gripper_links(self):
        links = []
        try:
            links = list(self.moveit.get_robot_model().get_joint_model_group("gripper").link_model_names)
        except Exception:  # noqa: BLE001
            pass
        links += ["robotiq_85_base_link", "robotiq_85_left_knuckle_link", "robotiq_85_right_knuckle_link",
                  "robotiq_85_left_finger_link", "robotiq_85_right_finger_link",
                  "robotiq_85_left_inner_knuckle_link", "robotiq_85_right_inner_knuckle_link",
                  "robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link",
                  "gripper_mount_link", "ur_to_robotiq_link", TOOL_LINK, "wrist_3_link"]
        return sorted(set(links))

    def detach_tray(self):
        aco = AttachedCollisionObject()
        aco.link_name = TOOL_LINK
        aco.object.id = "held_tray"
        aco.object.operation = CollisionObject.REMOVE
        self.aco_pub.publish(aco)
        time.sleep(0.3)
        # MoveIt drops a detached object back into the world where it was (around the fingers).
        # Delete that copy too, or every later plan starts "in collision" with it.
        rm = CollisionObject()
        rm.id = "held_tray"
        rm.header.frame_id = "world"
        rm.operation = CollisionObject.REMOVE
        self.co_pub.publish(rm)
        time.sleep(0.3)

    # ---------------- motion ----------------
    def state_from(self, q):
        st = RobotState(self.moveit.get_robot_model())
        st.set_to_default_values()
        st.set_joint_group_positions(ARM_GROUP, q)
        st.update()
        return st

    @staticmethod
    def wrap_near(v, ref):
        """Same joint angle +/- 2*pi, whichever is closest to ref (stays within UR's +/- 2*pi limits)."""
        best = v
        for k in (-1, 0, 1):
            c = v + 2 * math.pi * k
            if abs(c) <= 2 * math.pi and abs(c - ref) < abs(best - ref):
                best = c
        return best

    def ik_candidates(self, pose_stamped, tries=40, keep=6):
        """Collision-free IK solutions for tool0 at pose_stamped, closest to the current arm first."""
        q_now = self.current_arm_joints() or HOME
        seeds = [q_now, HOME] + [[random.uniform(-math.pi, math.pi) for _ in ARM_JOINTS]
                                 for _ in range(tries)]
        found = []
        with self.moveit.get_planning_scene_monitor().read_only() as scene:
            for seed in seeds:
                st = self.state_from(seed)
                if not st.set_from_ik(ARM_GROUP, pose_stamped.pose, TOOL_LINK, 0.05):
                    continue
                st.update()
                if scene.is_state_colliding(st, ARM_GROUP, False):
                    continue
                q = [float(v) for v in st.get_joint_group_positions(ARM_GROUP)]
                q = [self.wrap_near(v, r) if i != 2 else v   # skip elbow (limited to +/- pi)
                     for i, (v, r) in enumerate(zip(q, q_now))]
                if any(max(abs(a - b) for a, b in zip(q, f)) < 0.05 for f in found):
                    continue  # duplicate of one we already have
                found.append(q)
        found.sort(key=lambda q: sum((a - b) ** 2 for a, b in zip(q, q_now)))
        self.get_logger().info(f"IK: {len(found)} collision-free candidates")
        return found[:keep]

    def plan_once(self, set_goal, params):
        self.arm.set_start_state_to_current_state()
        set_goal()
        return self.arm.plan(single_plan_parameters=params)

    def move(self, goal, straight=False, label=""):
        if isinstance(goal, str):
            setters = [lambda: self.arm.set_goal_state(configuration_name=goal)]
        elif straight:
            setters = [lambda: self.arm.set_goal_state(pose_stamped_msg=goal, pose_link=TOOL_LINK)]
        else:
            cands = self.ik_candidates(goal)
            if not cands:
                raise RuntimeError(f"no collision-free IK: {label}")
            setters = [lambda q=q: self.arm.set_goal_state(robot_state=self.state_from(q)) for q in cands]

        primary = self.lin_params if straight else self.ptp_params
        result = None
        for s in setters:                       # try each IK solution with PTP/LIN first (fast)
            result = self.plan_once(s, primary)
            if result:
                break
        if not result:                          # then OMPL on the two best (slow, 5 s each)
            self.get_logger().warn(f"{label}: {'LIN' if straight else 'PTP'} plan failed, trying OMPL")
            for s in setters[:2]:
                result = self.plan_once(s, self.free_params)
                if result:
                    break
        if not result:
            raise RuntimeError(f"planning failed: {label}")

        # Where the trajectory ends, so we can confirm the real (Gazebo) arm actually got there
        jt = result.trajectory.get_robot_trajectory_msg().joint_trajectory
        goal_q = dict(zip(jt.joint_names, jt.points[-1].positions))
        dur = jt.points[-1].time_from_start.sec + jt.points[-1].time_from_start.nanosec * 1e-9
        try:
            status = self.moveit.execute(result.trajectory, controllers=[])
        except TypeError:  # older MoveItPy signature
            status = self.moveit.execute(result.trajectory, blocking=True, controllers=[])
        ok = "SUCCEEDED" if status is None else getattr(status, "status", status)
        if str(ok).upper() not in ("SUCCEEDED", "TRUE"):
            raise RuntimeError(f"execution failed ({ok}): {label}")
        self.wait_until_reached(goal_q, dur, label)

    def wait_until_reached(self, goal_q, duration, label, tol=0.02):
        """Block until /joint_states matches the trajectory's last point (execute() may return early)."""
        t0 = time.monotonic()
        limit = 3.0 * duration + 15.0   # Gazebo on WSL can run well below real time
        errs, err = {}, float("inf")
        while time.monotonic() - t0 < limit:
            errs = {j: self.arm_pos.get(j, 1e9) - q for j, q in goal_q.items() if j in ARM_JOINTS}
            err = max((abs(v) for v in errs.values()), default=0.0)
            if err < tol:
                time.sleep(0.2)  # let it settle
                return
            time.sleep(0.05)
        worst = max(errs, key=lambda j: abs(errs[j])) if errs else "?"
        target = [round(goal_q[j], 2) for j in ARM_JOINTS if j in goal_q]
        actual = [round(self.arm_pos.get(j, float("nan")), 2) for j in ARM_JOINTS]
        raise RuntimeError(f"arm did not reach goal: {label} (still {err:.3f} rad off; "
                           f"worst {worst} {errs.get(worst, 0.0):+.3f}) goal={target} actual={actual}")

    def set_gripper(self, position, label):
        if not self.gripper.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("gripper action server not available")
        goal = ParallelGripperCommand.Goal()
        goal.command.name = [GRIPPER_JOINT]
        goal.command.position = [float(position)]
        fut = self.gripper.send_goal_async(goal)
        self.wait(fut, 5.0, f"{label} (send)")
        handle = fut.result()
        if not handle.accepted:
            raise RuntimeError(f"gripper goal rejected: {label}")
        self.wait(handle.get_result_async(), float(self.get_parameter("gripper_timeout").value), label)
        time.sleep(0.3)

    @staticmethod
    def wait(fut, timeout, label):
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > timeout:
                raise RuntimeError(f"timeout: {label}")
            time.sleep(0.02)

    # ---------------- pick sequence ----------------
    def pick(self, target):
        gz = (target.pose.position.x, target.pose.position.y, target.pose.position.z)
        slot = min(SLOT_CENTERS, key=lambda s: sum((a - b) ** 2 for a, b in zip(SLOT_CENTERS[s], gz)))
        handle = (gz[0], gz[1] + HANDLE_FWD, gz[2] + HANDLE_UP + self.get_parameter("grasp_z_offset").value)
        pre = (handle[0], handle[1] + self.get_parameter("pregrasp_dist").value, handle[2])
        lift = self.get_parameter("lift_height").value
        lifted = (handle[0], handle[1], handle[2] + lift)
        out = (handle[0], handle[1] + self.get_parameter("retreat_dist").value, handle[2] + lift)
        px, py, ptop = self.get_parameter("place_xyz").value
        place = (px, py + HANDLE_FWD, ptop + 0.0875 + HANDLE_UP + 0.01)
        above_place = (place[0], place[1], place[2] + 0.08)

        self.get_logger().info(f"Picking tray in {slot}")
        self.pregrasp_joints = None
        self.reset_scene()
        self.ensure_home()
        self.set_gripper(self.get_parameter("pregrasp_gripper").value, "pre-open")
        self.move(self.tool_pose(pre), label="pre-grasp")
        time.sleep(0.3)
        self.pregrasp_joints = self.current_arm_joints()
        self.co_pub.publish(self.tray_object(slot, CollisionObject.REMOVE))
        time.sleep(0.3)
        self.move(self.tool_pose(handle), straight=True, label="approach")
        try:
            self.set_gripper(GRIPPER_CLOSED, "close")
        except RuntimeError as e:  # a slow/stalled gripper is not fatal; we check positions next
            self.get_logger().warn(str(e))
        pads = self.finger_pads_gz()   # where the fingers REALLY are (TF), not where we asked
        closed_fully = self.gripper_pos is not None and \
            self.gripper_pos > self.get_parameter("missed_grasp_threshold").value
        if closed_fully and self.get_parameter("require_finger_contact").value:
            raise RuntimeError(f"missed grasp (gripper closed to {self.gripper_pos:.3f})")
        bin_name, grasp_err = self.magnet_attach(pads)
        self.last_closure = self.gripper_pos
        self.attach_tray(slot)
        self.move(self.tool_pose(lifted), straight=True, label="lift")
        self.move(self.tool_pose(out), straight=True, label="retreat")
        self.move(self.tool_pose(above_place), label="to drop spot")
        self.move(self.tool_pose(place), straight=True, label="lower")
        self.magnet_release()
        self.set_gripper(GRIPPER_OPEN, "release")
        self.detach_tray()
        self.move(self.tool_pose(above_place), straight=True, label="clear")
        self.move("home", label="home")
        return bin_name, grasp_err

    def recover(self):
        """Get the arm back home from wherever it stopped, even if MoveIt thinks it is in collision."""
        self.magnet_release()
        try:
            self.set_gripper(GRIPPER_OPEN, "recover open")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"recover open: {e}")
        self.detach_tray()
        self.reset_scene()
        try:
            self.move("home", label="recover home")
            return
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"MoveIt recovery failed ({e}); backing out directly")
        # Straight back to where we were in front of the cubby, then home (no collision checking)
        waypoints = ([self.pregrasp_joints] if self.pregrasp_joints else []) + [HOME]
        self.move_joints_direct(waypoints, seconds_each=4.0)

    # ---------------- plumbing ----------------
    def on_target(self, msg):
        self.jobs.put(msg)

    def on_joints(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name == GRIPPER_JOINT:
                self.gripper_pos = pos
            elif name in ARM_JOINTS:
                self.arm_pos[name] = pos

    def worker(self):
        while rclpy.ok():
            target = self.jobs.get()
            self.publish_status("EXECUTING")
            t0 = time.monotonic()
            try:
                bin_name, err = self.pick(target)
                self.detail(f"ok in {time.monotonic() - t0:.1f}s | bin={bin_name} | grasp_err_cm={100 * err:.2f}"
                            f" | gripper_closed_to={self.last_closure:.3f}")
                self.publish_status("SUCCEEDED")
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(str(e))
                self.detail(str(e))
                self.publish_status("FAILED")
                try:
                    self.recover()
                except Exception as e2:  # noqa: BLE001
                    self.get_logger().error(f"recovery failed: {e2}")

    def publish_status(self, s):
        self.status_pub.publish(String(data=s))

    def detail(self, s):
        self.detail_pub.publish(String(data=s))


def main():
    rclpy.init()
    node = MedShelfManipulation()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

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
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import ParallelGripperCommand
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from std_msgs.msg import Empty, String

ARM_GROUP = "ur_manipulator"
TOOL_LINK = "tool0"
GRIPPER_JOINT = "robotiq_85_left_knuckle_joint"
GRIPPER_OPEN, GRIPPER_CLOSED = 0.0, 0.7929

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
        p("tcp_offset", 0.155)       # tool0 -> center of the finger pads (m). Tune in RViz.
        p("grasp_roll_deg", 0.0)     # rotate gripper about its approach axis if fingers close sideways
        p("pregrasp_dist", 0.12)     # stop this far in front of the handle before going in
        p("retreat_dist", 0.32)      # pull straight out this far (tray is 0.25 deep)
        p("place_xyz", [0.30, -0.20, 1.05])   # tray CENTER x,y and table-top z for the drop, Gazebo coords
        p("table_top_z", 1.04)       # top of the table collision box (just under the arm base)
        p("missed_grasp_threshold", 0.75)   # knuckle angle; closed on nothing ~0.79, on the 25 mm bar ~0.56
        p("attach_tolerance", 0.03)         # max gripper-to-handle distance (m) for the magnet to grab
        p("world_name", "medshelf")
        p("pregrasp_gripper", 0.40)         # partly open (~40 mm) so the lower finger fits the 20 mm handle slot

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.base = (g("base_x"), g("base_y"), g("base_z"))

        self.status_pub = self.create_publisher(String, "/medshelf/manipulation_status", 10)
        self.detail_pub = self.create_publisher(String, "/medshelf/manipulation_detail", 10)
        self.co_pub = self.create_publisher(CollisionObject, "/collision_object", 10)
        self.aco_pub = self.create_publisher(AttachedCollisionObject, "/attached_collision_object", 10)
        self.create_subscription(PoseStamped, "/medshelf/pick_target", self.on_target, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.gripper = ActionClient(self, ParallelGripperCommand, "/gripper_controller/gripper_cmd")

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
        return objs

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
            raise RuntimeError(f"no handle between fingers (nearest {b}, {100 * err:.1f} cm off)")
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

    # ---------------- motion ----------------
    def move(self, goal, straight=False, label=""):
        self.arm.set_start_state_to_current_state()
        if isinstance(goal, str):
            self.arm.set_goal_state(configuration_name=goal)
        else:
            self.arm.set_goal_state(pose_stamped_msg=goal, pose_link=TOOL_LINK)
        params = self.lin_params if straight else self.free_params
        result = self.arm.plan(single_plan_parameters=params)
        if not result and straight:  # fall back to free-space planning if LIN fails
            self.get_logger().warn(f"{label}: straight-line plan failed, trying OMPL")
            result = self.arm.plan(single_plan_parameters=self.free_params)
        if not result:
            raise RuntimeError(f"planning failed: {label}")
        try:
            status = self.moveit.execute(result.trajectory, controllers=[])
        except TypeError:  # older MoveItPy signature
            status = self.moveit.execute(result.trajectory, blocking=True, controllers=[])
        ok = "SUCCEEDED" if status is None else getattr(status, "status", status)
        if str(ok).upper() not in ("SUCCEEDED", "TRUE"):
            raise RuntimeError(f"execution failed ({ok}): {label}")

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
        self.wait(handle.get_result_async(), 10.0, label)
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
        handle = (gz[0], gz[1] + HANDLE_FWD, gz[2] + HANDLE_UP)
        pre = (handle[0], handle[1] + self.get_parameter("pregrasp_dist").value, handle[2])
        lifted = (handle[0], handle[1], handle[2] + 0.01)
        out = (handle[0], handle[1] + self.get_parameter("retreat_dist").value, handle[2] + 0.01)
        px, py, ptop = self.get_parameter("place_xyz").value
        place = (px, py + HANDLE_FWD, ptop + 0.0875 + HANDLE_UP + 0.01)
        above_place = (place[0], place[1], place[2] + 0.08)

        self.get_logger().info(f"Picking tray in {slot}")
        self.reset_scene()
        self.set_gripper(self.get_parameter("pregrasp_gripper").value, "pre-open")
        self.move(self.tool_pose(pre), label="pre-grasp")
        self.co_pub.publish(self.tray_object(slot, CollisionObject.REMOVE))
        time.sleep(0.3)
        self.move(self.tool_pose(handle), straight=True, label="approach")
        self.set_gripper(GRIPPER_CLOSED, "close")
        if self.gripper_pos is not None and self.gripper_pos > self.get_parameter("missed_grasp_threshold").value:
            self.set_gripper(GRIPPER_OPEN, "open after miss")
            self.move(self.tool_pose(pre), straight=True, label="back off after miss")
            raise RuntimeError(f"missed grasp (gripper closed to {self.gripper_pos:.3f})")
        bin_name, grasp_err = self.magnet_attach(handle)
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

    # ---------------- plumbing ----------------
    def on_target(self, msg):
        self.jobs.put(msg)

    def on_joints(self, msg):
        if GRIPPER_JOINT in msg.name:
            self.gripper_pos = msg.position[msg.name.index(GRIPPER_JOINT)]

    def worker(self):
        while rclpy.ok():
            target = self.jobs.get()
            self.publish_status("EXECUTING")
            t0 = time.monotonic()
            try:
                bin_name, err = self.pick(target)
                self.detail(f"ok in {time.monotonic() - t0:.1f}s | bin={bin_name} | grasp_err_cm={100 * err:.2f}")
                self.publish_status("SUCCEEDED")
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(str(e))
                self.detail(str(e))
                self.publish_status("FAILED")
                try:
                    self.magnet_release()
                    self.set_gripper(GRIPPER_OPEN, "recover open")
                    self.detach_tray()
                    self.move("home", label="recover home")
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

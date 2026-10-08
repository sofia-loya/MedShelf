#!/usr/bin/env python3
"""MedShelf trial runner.

Runs N trials end to end and writes one CSV row per trial:
  1. Shuffle the 9 trays into the 9 shelf cubbies in Gazebo (seeded, reproducible).
  2. Ask bin_detector for one supply (sets its `requested_bin` parameter).
  3. Wait for a confirmed detection and compare it to the ground-truth slot.
  4. (optional) Publish /medshelf/pick_target and wait for /medshelf/manipulation_status,
     logging joint tracking error from arm_controller while the arm moves.
  5. (optional) Ground-truth check in Gazebo: which tray actually ended up at the drop spot?

Modes:
  use_perception:=true   full pipeline (camera picks the cubby)
  use_perception:=false  manipulation-only: targets cycle through all 9 cubbies using the true
                         slot, so every cubby gets tested equally and perception errors are excluded

Uses only existing topics and does not modify any repo files. Needs running:
  gazebo.launch.py, camera.launch.py, and bin_detector.

Run:  python3 trial_runner.py --ros-args -p num_trials:=50 -p output_csv:=trials.csv
"""
import csv
import math
import random
import re
import subprocess
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from control_msgs.msg import JointTrajectoryControllerState

from my_arm_perception.bin_config import SLOT_NAMES, SUPPLY_TO_COLOR

# Ground-truth cubby positions in the Gazebo world (tray model origin = tray bottom center).
# "left/right" are as seen by the shelf camera / from the table, so left = +x.
SLOT_XYZ = {
    "top_left": (0.25, -0.75, 1.579), "top_center": (0.0, -0.75, 1.579), "top_right": (-0.25, -0.75, 1.579),
    "middle_left": (0.25, -0.75, 1.304), "middle_center": (0.0, -0.75, 1.304), "middle_right": (-0.25, -0.75, 1.304),
    "bottom_left": (0.25, -0.75, 1.029), "bottom_center": (0.0, -0.75, 1.029), "bottom_right": (-0.25, -0.75, 1.029),
}
CATEGORIES = list(SUPPLY_TO_COLOR)  # gauze, gloves, ...

CSV_FIELDS = [
    "trial", "seed", "requested", "expected_color", "gt_slot",
    "detected", "det_slot", "slot_correct", "found_category", "latency_s",
    "manip_status", "manip_time_s", "joint_rms_err", "joint_max_err",
    "picked_bin", "grasp_err_cm", "delivered_bin", "delivered_correct", "end_to_end",
    "manip_detail", "layout",
]


class TrialRunner(Node):
    def __init__(self):
        super().__init__("trial_runner")
        self.declare_parameter("num_trials", 20)
        self.declare_parameter("base_seed", 0)
        self.declare_parameter("output_csv", "trials.csv")
        self.declare_parameter("world_name", "medshelf")
        self.declare_parameter("detector_node", "/bin_detector")
        self.declare_parameter("settle_s", 1.5)
        self.declare_parameter("detect_timeout_s", 8.0)
        self.declare_parameter("run_manipulation", False)
        self.declare_parameter("manip_timeout_s", 120.0)
        self.declare_parameter("use_perception", True)
        self.declare_parameter("place_xyz", [0.15, -0.35, 1.10])  # must match medshelf_manipulation
        self.declare_parameter("delivery_radius", 0.12)

        self.lock = threading.Lock()
        self.visible = False
        self.color = "none"
        self.detection = None          # (slot, color, time) of first valid detection
        self.collecting = False
        self.expected_color = None
        self.manip_status = None
        self.joint_errors = []          # per-sample list of abs position errors
        self.recording_joints = False
        self.manip_detail = ""

        self.create_subscription(Bool, "/medshelf/target_visible", self.on_visible, 10)
        self.create_subscription(String, "/medshelf/detected_color", self.on_color, 10)
        self.create_subscription(String, "/medshelf/detected_slot", self.on_slot, 10)
        self.create_subscription(String, "/medshelf/manipulation_status", self.on_manip, 10)
        self.create_subscription(JointTrajectoryControllerState, "/arm_controller/controller_state",
                                 self.on_ctrl_state, 10)
        self.create_subscription(String, "/medshelf/manipulation_detail", self.on_detail, 10)
        self.pick_pub = self.create_publisher(PoseStamped, "/medshelf/pick_target", 10)

        detector = self.get_parameter("detector_node").value
        self.param_client = self.create_client(SetParameters, f"{detector}/set_parameters")

    # ---------- callbacks ----------
    def on_visible(self, msg):
        with self.lock:
            self.visible = msg.data

    def on_color(self, msg):
        with self.lock:
            self.color = msg.data

    def on_slot(self, msg):
        with self.lock:
            if (self.collecting and self.detection is None and self.visible
                    and msg.data != "none" and self.color == self.expected_color):
                self.detection = (msg.data, self.color, time.monotonic())

    def on_manip(self, msg):
        with self.lock:
            if msg.data in ("SUCCEEDED", "FAILED"):
                self.manip_status = msg.data

    def on_detail(self, msg):
        with self.lock:
            self.manip_detail = msg.data

    def on_ctrl_state(self, msg):
        if not self.recording_joints:
            return
        err = list(msg.error.positions) if msg.error.positions else []
        if err:
            with self.lock:
                self.joint_errors.append([abs(e) for e in err])

    # ---------- Gazebo helpers ----------
    def set_tray_pose(self, model, x, y, z):
        world = self.get_parameter("world_name").value
        # yaw = pi so the open side faces the table (quaternion z=1, w=0)
        req = (f'name: "{model}" position: {{x: {x} y: {y} z: {z}}} '
               f'orientation: {{x: 0 y: 0 z: 1 w: 0}}')
        for _attempt in range(3):  # Gazebo can be slow under load; retry before giving up
            res = subprocess.run(
                ["gz", "service", "-s", f"/world/{world}/set_pose",
                 "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                 "--timeout", "10000", "--req", req],
                capture_output=True, text=True)
            if "true" in res.stdout:
                return
        if "true" not in res.stdout:
            self.get_logger().warn(f"set_pose failed for {model}: {res.stdout.strip()} {res.stderr.strip()}")

    def gz_tray_poses(self):
        world = self.get_parameter("world_name").value
        try:
            out = subprocess.run(["gz", "topic", "-e", "-n", "1", "-t", f"/world/{world}/pose/info"],
                                 capture_output=True, text=True, timeout=10).stdout
        except subprocess.TimeoutExpired:
            return {}
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

    def delivered_tray(self):
        """Which tray (if any) is sitting at the drop spot on the table."""
        px, py, ptop = self.get_parameter("place_xyz").value
        radius = float(self.get_parameter("delivery_radius").value)
        best, best_d = "none", radius
        for b, (x, y, z) in self.gz_tray_poses().items():
            d = math.hypot(x - px, y - py)
            if d < best_d and abs(z - ptop) < 0.15:
                best, best_d = b, d
        return best

    def apply_layout(self, layout):
        # Park every tray off the shelf first so no two trays ever overlap mid-shuffle.
        for i, cat in enumerate(CATEGORIES):
            self.set_tray_pose(f"tray_{cat}", 3.0 + 0.4 * i, 3.0, 0.01)
        for slot, cat in layout.items():
            x, y, z = SLOT_XYZ[slot]
            self.set_tray_pose(f"tray_{cat}", x, y, z)

    def set_requested_bin(self, category):
        if not self.param_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("bin_detector set_parameters service not available. Is bin_detector running?")
        req = SetParameters.Request()
        req.parameters = [Parameter("requested_bin", Parameter.Type.STRING, category).to_parameter_msg()]
        fut = self.param_client.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 5.0:
            time.sleep(0.05)

    # ---------- one trial ----------
    def run_trial(self, trial, seed):
        rng = random.Random(seed)
        cats = CATEGORIES[:]
        rng.shuffle(cats)
        layout = dict(zip(SLOT_NAMES, cats))           # slot -> category
        use_perception = bool(self.get_parameter("use_perception").value)
        if use_perception:
            requested = rng.choice(CATEGORIES)
            gt_slot = next(s for s, c in layout.items() if c == requested)
        else:
            gt_slot = SLOT_NAMES[trial % len(SLOT_NAMES)]   # sweep every cubby evenly
            requested = layout[gt_slot]
        expected_color = SUPPLY_TO_COLOR[requested]

        self.apply_layout(layout)
        if use_perception:
            self.set_requested_bin(requested)
        time.sleep(float(self.get_parameter("settle_s").value))

        with self.lock:
            self.expected_color = expected_color
            self.detection = None
            self.manip_status = None
            self.collecting = True
        t_start = time.monotonic()
        timeout = float(self.get_parameter("detect_timeout_s").value) if use_perception else 0.0
        if not use_perception:
            with self.lock:
                self.detection = (gt_slot, expected_color, t_start)
        while time.monotonic() - t_start < timeout:
            with self.lock:
                if self.detection is not None:
                    break
            time.sleep(0.05)
        with self.lock:
            self.collecting = False
            det = self.detection

        row = {
            "trial": trial, "seed": seed, "requested": requested,
            "expected_color": expected_color, "gt_slot": gt_slot,
            "detected": det is not None,
            "det_slot": det[0] if det else "none",
            "slot_correct": bool(det and det[0] == gt_slot),
            "found_category": layout.get(det[0], "none") if det else "none",
            "latency_s": round(det[2] - t_start, 3) if det else "",
            "manip_status": "not_run", "manip_time_s": "",
            "joint_rms_err": "", "joint_max_err": "",
            "picked_bin": "", "grasp_err_cm": "", "delivered_bin": "",
            "delivered_correct": "", "end_to_end": "", "manip_detail": "",
            "layout": ";".join(f"{s}:{c}" for s, c in layout.items()),
        }

        if det and self.get_parameter("run_manipulation").value:
            row.update(self.run_manipulation(det[0]))
            time.sleep(1.0)  # let the released tray settle on the table
            delivered = self.delivered_tray()
            row["delivered_bin"] = delivered
            row["delivered_correct"] = delivered == requested
            row["end_to_end"] = bool(row["slot_correct"] and delivered == requested)
            m = re.search(r"bin=([a-z_]+)", row["manip_detail"])
            row["picked_bin"] = m.group(1) if m else ""
            m = re.search(r"grasp_err_cm=([\d.]+)", row["manip_detail"])
            row["grasp_err_cm"] = m.group(1) if m else ""

        self.get_logger().info(
            f"[trial {trial}] want {requested} ({expected_color}) at {gt_slot} -> "
            f"detected {row['det_slot']} {'OK' if row['slot_correct'] else 'WRONG'} | manip {row['manip_status']}"
            f" | delivered {row['delivered_bin'] or '-'} | {row['manip_detail']}")
        return row

    def run_manipulation(self, det_slot):
        x, y, z = SLOT_XYZ[det_slot]
        target = PoseStamped()
        target.header.frame_id = "world"
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x, target.pose.position.y, target.pose.position.z = x, y, z + 0.0875
        target.pose.orientation.w = 1.0

        with self.lock:
            self.joint_errors = []
            self.manip_status = None
            self.manip_detail = ""
        self.recording_joints = True
        t0 = time.monotonic()
        self.pick_pub.publish(target)
        timeout = float(self.get_parameter("manip_timeout_s").value)
        while time.monotonic() - t0 < timeout:
            with self.lock:
                if self.manip_status is not None:
                    break
            time.sleep(0.05)
        self.recording_joints = False

        time.sleep(0.3)  # detail is published just before the final status
        with self.lock:
            status = self.manip_status or "TIMEOUT"
            errs = self.joint_errors[:]
            detail = self.manip_detail
        flat = [e for sample in errs for e in sample]
        rms = math.sqrt(sum(e * e for e in flat) / len(flat)) if flat else ""
        mx = max(flat) if flat else ""
        return {
            "manip_status": status,
            "manip_time_s": round(time.monotonic() - t0, 2),
            "joint_rms_err": round(rms, 5) if flat else "",
            "joint_max_err": round(mx, 5) if flat else "",
            "manip_detail": detail,
        }


def main():
    rclpy.init()
    node = TrialRunner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    n = int(node.get_parameter("num_trials").value)
    base_seed = int(node.get_parameter("base_seed").value)
    out = node.get_parameter("output_csv").value
    node.get_logger().info(f"Running {n} trials -> {out}")

    try:
        with open(out, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            writer.writeheader()
            for i in range(n):
                writer.writerow(node.run_trial(i, base_seed + i))
                f.flush()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"Done. Results in {out}. Run: python3 analyze_trials.py {out}")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

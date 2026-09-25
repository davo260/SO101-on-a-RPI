"""so101_bridge - publishes the SO-101 state from so101d's shared memory to ROS 2.

Topics
  /joint_states                  sensor_msgs/JointState   follower (pos rad, vel rad/s, effort = load %)
  /so101/leader/joint_states     sensor_msgs/JointState   leader (pos rad)
  /so101/status                  std_msgs/String          JSON: mode, faults, timing (1 Hz)
  /so101/cmd  (subscribed)       std_msgs/String          idle|teleop|hold|estop|reset

A plain (non real-time) shared-memory client, like so101ctl or so101_log: it
never disturbs the 100 Hz control loop. If so101d is not running yet, or
restarts (systemd watchdog, reboot), the node keeps running and re-attaches to
the new shared-memory segment by itself. Needs so101.py (clients/python) on
PYTHONPATH and read access to /dev/shm/so101 (group so101).
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from so101 import So101

from .convert import JOINTS, TickToRad, load_calibration

TICKS_TO_RAD = 2.0 * math.pi / 4095.0


class So101Bridge(Node):
    def __init__(self):
        super().__init__("so101_bridge")
        p = self.declare_parameter
        rate = p("rate_hz", 50.0).value
        f_cal = p("follower_calibration", "/etc/so101/so101_follower.json").value
        l_cal = p("leader_calibration", "/etc/so101/so101_leader.json").value
        self.f_conv = TickToRad(load_calibration(f_cal),
                                p("follower_signs", [1.0] * 6).value,
                                p("follower_offsets", [0.0] * 6).value,
                                p("follower_gripper_rad", [0.0, 1.745]).value)
        self.l_conv = TickToRad(load_calibration(l_cal),
                                p("leader_signs", [1.0] * 6).value,
                                p("leader_offsets", [0.0] * 6).value,
                                p("leader_gripper_rad", [0.0, 0.785]).value)
        self.arm = None
        self._next_attach = 0.0
        self._attach()
        self.pub_f = self.create_publisher(JointState, "joint_states", 10)
        self.pub_l = self.create_publisher(JointState, "so101/leader/joint_states", 10)
        self.pub_status = self.create_publisher(String, "so101/status", 10)
        self.create_subscription(String, "so101/cmd", self.on_cmd, 10)
        self.create_timer(1.0 / rate, self.tick)
        self.create_timer(1.0, self.status)
        self.last_cycle = None
        self.rate = rate

    def _attach(self):
        """(Re)attach to /dev/shm/so101; retried at most once per second."""
        now = time.monotonic()
        if now < self._next_attach:
            return False
        self._next_attach = now + 1.0
        try:
            arm = So101()
        except (FileNotFoundError, RuntimeError, PermissionError) as e:
            if self.arm is not None or not getattr(self, "_warned", False):
                self.get_logger().warn(f"waiting for so101d ({e})")
                self._warned = True
            self.arm = None
            return False
        if self.arm is not None:
            self.arm.close()
        self.arm = arm
        self._warned = False
        self.last_cycle = None
        self.get_logger().info(
            f"attached to so101d pid {arm.pid}, publishing /joint_states at "
            f"{self.get_parameter('rate_hz').value:.0f} Hz")
        return True

    def _ready(self):
        if self.arm is not None and self.arm.alive():
            return True
        return self._attach()          # daemon absent or restarted

    def tick(self):
        if not self._ready():
            return
        st = self.arm.read()
        if st["cycle"] == self.last_cycle:
            return                                    # nothing new
        self.last_cycle = st["cycle"]
        stamp = self.get_clock().now().to_msg()
        if st["follower"]["ok"]:
            f = st["follower"]
            m = JointState()
            m.header.stamp = stamp
            m.name = JOINTS
            m.position = self.f_conv(f["pos"])
            m.velocity = [s * v * TICKS_TO_RAD for s, v in zip(self.f_conv.signs, f["vel"])]
            m.effort = [x / 10.0 for x in f["load"]]  # % of max torque
            self.pub_f.publish(m)
        if st["leader"]["ok"]:
            m = JointState()
            m.header.stamp = stamp
            m.name = JOINTS
            m.position = self.l_conv(st["leader"]["pos"])
            self.pub_l.publish(m)

    def status(self):
        if not self._ready():
            return
        st = self.arm.read()
        self.pub_status.publish(String(data=json.dumps({
            "cycle": st["cycle"], "mode": st["mode"], "torque": st["torque"],
            "ramping": st["ramping"], "faults": st["faults"],
            "timing_us": st["timing_us"], "counters": st["counters"],
            "temp": st["follower"]["temp"], "volt": st["follower"]["volt"]})))

    def on_cmd(self, msg):
        name = msg.data.strip().lower()
        if not self._ready():
            self.get_logger().warn(f"cmd {name} ignored: so101d not running")
            return
        try:
            res = self.arm.command(name)
        except Exception as e:                        # bad name, permissions, timeout
            res = f"ERROR {e}"
        self.get_logger().info(f"cmd {name} -> {res}")


def main():
    rclpy.init()
    node = So101Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

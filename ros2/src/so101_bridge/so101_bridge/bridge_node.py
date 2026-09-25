"""so101_bridge - publishes the SO-101 state from so101d's shared memory to ROS 2.

Topics
  /joint_states                  sensor_msgs/JointState   follower (pos rad, vel rad/s, effort = load %)
  /so101/leader/joint_states     sensor_msgs/JointState   leader (pos rad)
  /so101/status                  std_msgs/String          JSON: mode, faults, timing (1 Hz)
  /so101/cmd  (subscribed)       std_msgs/String          idle|teleop|hold|estop|reset

A plain (non real-time) shared-memory client, like so101ctl or so101_log: it
never disturbs the 100 Hz control loop. Needs so101.py (clients/python) on
PYTHONPATH and read access to /dev/shm/so101 (group so101).
"""
import json
import math

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
                                p("follower_offsets", [0.0] * 6).value)
        self.l_conv = TickToRad(load_calibration(l_cal),
                                p("leader_signs", [1.0] * 6).value,
                                p("leader_offsets", [0.0] * 6).value)
        self.arm = So101()
        self.pub_f = self.create_publisher(JointState, "joint_states", 10)
        self.pub_l = self.create_publisher(JointState, "so101/leader/joint_states", 10)
        self.pub_status = self.create_publisher(String, "so101/status", 10)
        self.create_subscription(String, "so101/cmd", self.on_cmd, 10)
        self.create_timer(1.0 / rate, self.tick)
        self.create_timer(1.0, self.status)
        self.last_cycle = None
        self.get_logger().info(
            f"so101d pid {self.arm.pid}, publishing /joint_states at {rate:.0f} Hz")

    def tick(self):
        if not self.arm.alive():
            self.get_logger().error("so101d stopped", throttle_duration_sec=5.0)
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
        if not self.arm.alive():
            return
        st = self.arm.read()
        self.pub_status.publish(String(data=json.dumps({
            "cycle": st["cycle"], "mode": st["mode"], "torque": st["torque"],
            "ramping": st["ramping"], "faults": st["faults"],
            "timing_us": st["timing_us"], "counters": st["counters"],
            "temp": st["follower"]["temp"], "volt": st["follower"]["volt"]})))

    def on_cmd(self, msg):
        name = msg.data.strip().lower()
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

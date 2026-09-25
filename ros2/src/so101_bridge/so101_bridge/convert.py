"""Raw servo ticks -> URDF joint angles (radians), no ROS dependency.

The SO-101 URDF ("new calibration", TheRobotStudio) puts each joint's zero at
the MIDDLE of its calibrated range, the same convention as LeRobot's degrees
mode:   angle = (raw - (range_min + range_max) / 2) * 2*pi / 4095
A per-joint sign and offset (radians) absorb any remaining mismatch between
the physical arm and the model; tune them once with the arm in a known pose.

The GRIPPER is different: the URDF does not put its zero at mid-range (the
jaw is closed near 0 rad). LeRobot treats it as 0..100 (closed..open), so it
is mapped linearly: range_min -> gripper_rad[0] (closed), range_max ->
gripper_rad[1] (open). Follower URDF: 0 .. 1.745 rad; leader trigger 0 .. 0.785.
"""
import json
import math

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper"]
TICKS_PER_REV = 4095.0


def load_calibration(path):
    """LeRobot calibration JSON -> list of (range_min, range_max) in JOINTS order."""
    with open(path) as f:
        cal = json.load(f)
    return [(cal[j]["range_min"], cal[j]["range_max"]) for j in JOINTS]


class TickToRad:
    def __init__(self, ranges, signs=None, offsets=None, gripper_rad=None):
        self.ranges = list(ranges)
        self.mid = [(lo + hi) / 2.0 for lo, hi in ranges]
        self.signs = list(signs) if signs else [1.0] * len(ranges)
        self.offsets = list(offsets) if offsets else [0.0] * len(ranges)
        self.gripper_rad = tuple(gripper_rad) if gripper_rad else None

    def __call__(self, raw):
        k = 2.0 * math.pi / TICKS_PER_REV
        out = [s * (r - m) * k + o
               for r, m, s, o in zip(raw, self.mid, self.signs, self.offsets)]
        if self.gripper_rad is not None:           # last joint = gripper
            lo, hi = self.ranges[-1]
            u = min(max((raw[-1] - lo) / (hi - lo), 0.0), 1.0)
            closed, opened = self.gripper_rad
            out[-1] = closed + u * (opened - closed) + self.offsets[-1]
        return out

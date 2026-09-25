"""Raw servo ticks -> URDF joint angles (radians), no ROS dependency.

The SO-101 URDF ("new calibration", TheRobotStudio) puts each joint's zero at
the MIDDLE of its calibrated range, the same convention as LeRobot's degrees
mode:   angle = (raw - (range_min + range_max) / 2) * 2*pi / 4095
A per-joint sign and offset (radians) absorb any remaining mismatch between
the physical arm and the model; tune them once with the arm in a known pose.
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
    def __init__(self, ranges, signs=None, offsets=None):
        self.mid = [(lo + hi) / 2.0 for lo, hi in ranges]
        self.signs = list(signs) if signs else [1.0] * len(ranges)
        self.offsets = list(offsets) if offsets else [0.0] * len(ranges)

    def __call__(self, raw):
        k = 2.0 * math.pi / TICKS_PER_REV
        return [s * (r - m) * k + o
                for r, m, s, o in zip(raw, self.mid, self.signs, self.offsets)]

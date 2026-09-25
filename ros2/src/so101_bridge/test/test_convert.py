import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from so101_bridge.convert import TickToRad, load_calibration  # noqa: E402

FIX = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..",
                   "daemon", "tests", "fixtures", "so101_leader.json")


def test_mid_is_zero():
    ranges = load_calibration(FIX)
    conv = TickToRad(ranges)
    mids = [(lo + hi) / 2 for lo, hi in ranges]
    assert all(abs(a) < 1e-9 for a in conv(mids))


def test_quarter_turn_and_sign():
    conv = TickToRad([(0, 4095)] * 6, signs=[1, -1, 1, 1, 1, 1], offsets=[0, 0, 0.1, 0, 0, 0])
    out = conv([2047.5 + 4095 / 4] * 6)
    assert abs(out[0] - math.pi / 2) < 1e-9
    assert abs(out[1] + math.pi / 2) < 1e-9
    assert abs(out[2] - (math.pi / 2 + 0.1)) < 1e-9

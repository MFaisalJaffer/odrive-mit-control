#!/usr/bin/env python3
"""
Unit tests for leg_test.py pure-logic helpers.

These tests don't touch the CAN bus — they verify the math used for
position normalization, turn-offset compensation, MIT bit packing,
URDF parsing, and range checking.

Run:
  python3 -m unittest test_leg_test.py -v
  # or
  python3 test_leg_test.py
"""

import io
import math
import os
import struct
import sys
import tempfile
import unittest

# Make the script importable as a module
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import leg_test as lt


# ── wrap_to_pi ────────────────────────────────────────────────────────────────
class TestWrapToPi(unittest.TestCase):
    def assertClose(self, a, b, tol=1e-9):
        self.assertLess(abs(a - b), tol, msg=f"{a} not close to {b}")

    def test_zero_unchanged(self):
        self.assertClose(lt.wrap_to_pi(0.0), 0.0)

    def test_small_positive_unchanged(self):
        self.assertClose(lt.wrap_to_pi(1.5), 1.5)

    def test_small_negative_unchanged(self):
        self.assertClose(lt.wrap_to_pi(-1.5), -1.5)

    def test_full_revolution_wraps_to_zero(self):
        self.assertClose(lt.wrap_to_pi(2 * math.pi), 0.0, tol=1e-12)

    def test_negative_full_revolution_wraps_to_zero(self):
        self.assertClose(lt.wrap_to_pi(-2 * math.pi), 0.0, tol=1e-12)

    def test_one_and_a_quarter_revolution(self):
        # +2π + π/2  → +π/2
        self.assertClose(lt.wrap_to_pi(2 * math.pi + math.pi / 2),
                         math.pi / 2)

    def test_observed_drift_6_277(self):
        # Real value observed in hardware testing
        self.assertClose(lt.wrap_to_pi(6.277), 6.277 - 2 * math.pi, tol=1e-9)

    def test_observed_drift_7_849(self):
        self.assertClose(lt.wrap_to_pi(7.849), 7.849 - 2 * math.pi, tol=1e-9)

    def test_at_positive_pi_boundary(self):
        # Exactly +π wraps to -π (or +π — boundary is ambiguous; our impl
        # uses modulo so +π → +π - 2π = -π)
        result = lt.wrap_to_pi(math.pi)
        self.assertTrue(abs(result - math.pi) < 1e-9 or
                        abs(result + math.pi) < 1e-9)

    def test_none_passes_through(self):
        self.assertIsNone(lt.wrap_to_pi(None))

    def test_output_range(self):
        # Sweep many inputs and verify all results in [-π, +π]
        for x in [-100.0, -10.5, -math.pi, 0.0, math.pi, 10.5, 100.0]:
            w = lt.wrap_to_pi(x)
            self.assertGreaterEqual(w, -math.pi - 1e-9)
            self.assertLessEqual(w, math.pi + 1e-9)


# ── compute_turn_offsets ──────────────────────────────────────────────────────
class TestComputeTurnOffsets(unittest.TestCase):
    def test_no_offset_when_in_single_turn(self):
        offsets = lt.compute_turn_offsets({1: 0.5, 2: -1.2})
        self.assertAlmostEqual(offsets[1], 0.0, places=9)
        self.assertAlmostEqual(offsets[2], 0.0, places=9)

    def test_one_revolution_offset(self):
        # raw = 2π + 0.5 → offset = 2π, wrapped = 0.5
        raw = 2 * math.pi + 0.5
        offsets = lt.compute_turn_offsets({1: raw})
        self.assertAlmostEqual(offsets[1], 2 * math.pi, places=9)

    def test_negative_revolution_offset(self):
        raw = -2 * math.pi - 0.5
        offsets = lt.compute_turn_offsets({1: raw})
        self.assertAlmostEqual(offsets[1], -2 * math.pi, places=9)

    def test_offset_plus_wrap_reconstructs_raw(self):
        # The whole point: cmd_raw = wrapped + offset must equal the original raw
        for raw in [0.5, -0.5, 6.277, 7.849, -6.277, 3.14, -3.14]:
            offsets = lt.compute_turn_offsets({1: raw})
            wrapped = lt.wrap_to_pi(raw)
            reconstructed = wrapped + offsets[1]
            self.assertAlmostEqual(reconstructed, raw, places=9,
                                   msg=f"raw={raw}: got {reconstructed}")

    def test_skips_none_entries(self):
        offsets = lt.compute_turn_offsets({1: 0.5, 2: None})
        self.assertIn(1, offsets)
        self.assertNotIn(2, offsets)


# ── smoothstep ────────────────────────────────────────────────────────────────
class TestSmoothstep(unittest.TestCase):
    def test_endpoints(self):
        self.assertEqual(lt.smoothstep(0.0), 0.0)
        self.assertEqual(lt.smoothstep(1.0), 1.0)

    def test_midpoint(self):
        self.assertAlmostEqual(lt.smoothstep(0.5), 0.5, places=9)

    def test_clamps_below_zero(self):
        self.assertEqual(lt.smoothstep(-0.5), 0.0)

    def test_clamps_above_one(self):
        self.assertEqual(lt.smoothstep(1.5), 1.0)

    def test_monotonic(self):
        prev = lt.smoothstep(0.0)
        for t in [i / 100 for i in range(1, 101)]:
            cur = lt.smoothstep(t)
            self.assertGreaterEqual(cur, prev)
            prev = cur


# ── float_to_uint (MIT bit packing) ───────────────────────────────────────────
class TestFloatToUint(unittest.TestCase):
    def test_zero_at_center(self):
        # 0 is the midpoint of [-12.5, +12.5] → ~half of 2^16
        v = lt.float_to_uint(0.0, -12.5, 12.5, 16)
        self.assertAlmostEqual(v, 32767, delta=1)

    def test_minimum(self):
        self.assertEqual(lt.float_to_uint(-12.5, -12.5, 12.5, 16), 0)

    def test_maximum(self):
        self.assertEqual(lt.float_to_uint(12.5, -12.5, 12.5, 16), 65535)

    def test_clamps_below_min(self):
        self.assertEqual(lt.float_to_uint(-100.0, -12.5, 12.5, 16), 0)

    def test_clamps_above_max(self):
        self.assertEqual(lt.float_to_uint(100.0, -12.5, 12.5, 16), 65535)

    def test_kp_zero(self):
        # kp range is [0, 500], 0 → uint 0
        self.assertEqual(lt.float_to_uint(0.0, 0.0, 500.0, 12), 0)

    def test_kp_max(self):
        self.assertEqual(lt.float_to_uint(500.0, 0.0, 500.0, 12), 4095)


# ── can_id ────────────────────────────────────────────────────────────────────
class TestCanId(unittest.TestCase):
    def test_node_3_mit(self):
        # (3 << 5) | 0x008 = 0x68
        self.assertEqual(lt.can_id(3, 0x008), 0x068)

    def test_node_5_enc_est(self):
        # (5 << 5) | 0x009 = 0xA9
        self.assertEqual(lt.can_id(5, 0x009), 0xA9)


# ── parse_urdf_limits ─────────────────────────────────────────────────────────
SAMPLE_URDF = """<?xml version="1.0"?>
<robot name="test">
  <joint name="joint_a" type="revolute">
    <limit effort="100" velocity="10" lower="-1.5" upper="1.5"/>
  </joint>
  <joint name="joint_b" type="revolute">
    <limit effort="100" velocity="10" lower="-2.0" upper="0.5"/>
  </joint>
  <joint name="joint_no_limit" type="fixed"/>
  <ros2_control name="hw" type="system">
    <!-- These duplicates have no <limit>; parser must skip them -->
    <joint name="joint_a">
      <param name="node_id">1</param>
    </joint>
    <joint name="joint_b">
      <param name="node_id">2</param>
    </joint>
  </ros2_control>
</robot>
"""


class TestParseUrdfLimits(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.urdf',
                                                delete=False)
        self.tmp.write(SAMPLE_URDF)
        self.tmp.close()

    def tearDown(self):
        os.unlink(self.tmp.name)

    def test_parses_known_joints(self):
        limits = lt.parse_urdf_limits(self.tmp.name, ['joint_a', 'joint_b'])
        self.assertEqual(limits['joint_a'], (-1.5, 1.5))
        self.assertEqual(limits['joint_b'], (-2.0, 0.5))

    def test_ignores_ros2_control_duplicates(self):
        # The ros2_control joints have no <limit> but parser must succeed by
        # only looking at top-level <joint> elements
        limits = lt.parse_urdf_limits(self.tmp.name, ['joint_a'])
        self.assertEqual(limits['joint_a'], (-1.5, 1.5))

    def test_missing_joint_raises(self):
        with self.assertRaises(ValueError):
            lt.parse_urdf_limits(self.tmp.name, ['no_such_joint'])

    def test_joint_without_limit_raises(self):
        with self.assertRaises(ValueError):
            lt.parse_urdf_limits(self.tmp.name, ['joint_no_limit'])


# ── check_within_limits ───────────────────────────────────────────────────────
class TestCheckWithinLimits(unittest.TestCase):
    def _silent(self, fn, *args, **kwargs):
        """Run fn while suppressing its stdout output (so test runs are quiet)."""
        old = sys.stdout
        sys.stdout = io.StringIO()
        try:
            return fn(*args, **kwargs)
        finally:
            sys.stdout = old

    def test_all_within_range_returns_true(self):
        positions = {1: 0.0, 2: -1.0}
        limits    = {1: (-1.5, 1.5), 2: (-2.0, 0.5)}
        names     = {1: 'a', 2: 'b'}
        ok = self._silent(lt.check_within_limits, positions, limits, names)
        self.assertTrue(ok)

    def test_one_outside_returns_false(self):
        positions = {1: 2.0, 2: -1.0}    # joint 1 is above its upper (1.5)
        limits    = {1: (-1.5, 1.5), 2: (-2.0, 0.5)}
        names     = {1: 'a', 2: 'b'}
        ok = self._silent(lt.check_within_limits, positions, limits, names)
        self.assertFalse(ok)

    def test_missing_reading_returns_false(self):
        positions = {1: 0.0}            # joint 2 has no reading
        limits    = {1: (-1.5, 1.5), 2: (-2.0, 0.5)}
        names     = {1: 'a', 2: 'b'}
        ok = self._silent(lt.check_within_limits, positions, limits, names)
        self.assertFalse(ok)

    def test_safety_margin_allows_just_outside(self):
        # SAFETY_MARGIN is 0.05; a reading 0.04 above upper should still pass
        positions = {1: 1.5 + 0.04}
        limits    = {1: (-1.5, 1.5)}
        names     = {1: 'a'}
        ok = self._silent(lt.check_within_limits, positions, limits, names)
        self.assertTrue(ok)

    def test_safety_margin_does_not_allow_too_far(self):
        # 0.06 above upper exceeds the 0.05 margin
        positions = {1: 1.5 + 0.06}
        limits    = {1: (-1.5, 1.5)}
        names     = {1: 'a'}
        ok = self._silent(lt.check_within_limits, positions, limits, names)
        self.assertFalse(ok)


# ── MIT round-trip: pack then unpack ──────────────────────────────────────────
class TestMitRoundTrip(unittest.TestCase):
    def _uint_to_float(self, x_int, x_min, x_max, bits):
        return x_int * (x_max - x_min) / ((1 << bits) - 1) + x_min

    def test_position_round_trip_0(self):
        v = lt.float_to_uint(0.0, lt.MIT_P_MIN, lt.MIT_P_MAX, 16)
        back = self._uint_to_float(v, lt.MIT_P_MIN, lt.MIT_P_MAX, 16)
        self.assertAlmostEqual(back, 0.0, places=3)

    def test_position_round_trip_pi(self):
        v = lt.float_to_uint(math.pi, lt.MIT_P_MIN, lt.MIT_P_MAX, 16)
        back = self._uint_to_float(v, lt.MIT_P_MIN, lt.MIT_P_MAX, 16)
        self.assertAlmostEqual(back, math.pi, places=3)

    def test_kp_round_trip(self):
        for kp in [0.0, 50.0, 100.0, 250.0, 500.0]:
            v = lt.float_to_uint(kp, lt.MIT_KP_MIN, lt.MIT_KP_MAX, 12)
            back = self._uint_to_float(v, lt.MIT_KP_MIN, lt.MIT_KP_MAX, 12)
            self.assertAlmostEqual(back, kp, delta=0.2)


# ── LEGS map sanity ───────────────────────────────────────────────────────────
class TestLegsMap(unittest.TestCase):
    def test_both_legs_present(self):
        self.assertIn('right', lt.LEGS)
        self.assertIn('left', lt.LEGS)

    def test_each_leg_has_5_joints(self):
        for leg in ('right', 'left'):
            self.assertEqual(len(lt.LEGS[leg]['joints']), 5)

    def test_node_ids_are_unique_within_leg(self):
        for leg in ('right', 'left'):
            ids = [j[0] for j in lt.LEGS[leg]['joints']]
            self.assertEqual(len(ids), len(set(ids)),
                             f"Duplicate node IDs in {leg} leg")

    def test_node_ids_disjoint_across_legs(self):
        r = {j[0] for j in lt.LEGS['right']['joints']}
        l = {j[0] for j in lt.LEGS['left']['joints']}
        self.assertEqual(r & l, set(),
                         "Right and left legs share node IDs")

    def test_can_interface_matches_convention(self):
        # right=can0, left=can1 per repo convention
        self.assertEqual(lt.LEGS['right']['can'], 'can0')
        self.assertEqual(lt.LEGS['left']['can'], 'can1')


if __name__ == '__main__':
    unittest.main(verbosity=2)

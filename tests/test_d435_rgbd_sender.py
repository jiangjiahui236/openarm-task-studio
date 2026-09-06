import importlib.util
import sys
import unittest
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).parents[1] / "d435_rgbd_sender.py"
SPEC = importlib.util.spec_from_file_location("task_studio_rgbd_sender_test", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def observation(shoulder=(0.0, 0.0, 1.0), elbow=(0.0, 0.25, 1.0), wrist=(0.0, 0.50, 1.0)):
    shoulder = np.array(shoulder, dtype=float)
    elbow = np.array(elbow, dtype=float)
    wrist = np.array(wrist, dtype=float)
    return MODULE.ArmObservation(
        shoulder=shoulder,
        elbow=elbow,
        wrist=wrist,
        confidence=0.9,
        forearm_dir=MODULE.normalize(wrist - elbow),
        upper_arm_dir=MODULE.normalize(elbow - shoulder),
    )


class ArmObservationStabilizerTest(unittest.TestCase):
    def setUp(self):
        self.stabilizer = MODULE.ArmObservationStabilizer({"tracking": {}})

    def test_valid_motion_is_smoothed_and_updates_segment_reference(self):
        first = self.stabilizer.update({"left": observation()}, 1.0)["left"]
        moved = observation(wrist=(0.02, 0.50, 1.0))
        second = self.stabilizer.update({"left": moved}, 1.04)["left"]
        self.assertEqual(second.tracking_status, "live")
        self.assertGreater(second.wrist[0], first.wrist[0])
        self.assertLess(second.wrist[0], moved.wrist[0])
        self.assertIsNotNone(self.stabilizer.segment_reference["left"])

    def test_implausible_background_depth_is_rejected(self):
        accepted = self.stabilizer.update({"left": observation()}, 1.0)["left"]
        bad = observation(wrist=(0.0, 0.50, 2.5))
        held = self.stabilizer.update({"left": bad}, 1.03)["left"]
        self.assertEqual(held.tracking_status, "held:geometry")
        np.testing.assert_allclose(held.wrist, accepted.wrist)

    def test_short_depth_dropout_is_held_then_expires(self):
        accepted = self.stabilizer.update({"left": observation()}, 1.0)["left"]
        held = self.stabilizer.update({}, 1.10)["left"]
        self.assertEqual(held.tracking_status, "held:depth")
        np.testing.assert_allclose(held.elbow, accepted.elbow)
        self.assertNotIn("left", self.stabilizer.update({}, 1.16))

    def test_configured_fast_human_motion_is_accepted(self):
        stabilizer = MODULE.ArmObservationStabilizer({
            "tracking": {
                "max_point_speed_m_s": 20.0,
                "point_jump_allowance_m": 0.08,
                "position_smoothing_alpha": 0.95,
            }
        })
        stabilizer.update({"left": observation()}, 1.0)
        fast = observation(
            shoulder=(0.50, 0.0, 1.0),
            elbow=(0.50, 0.25, 1.0),
            wrist=(0.50, 0.50, 1.0),
        )
        accepted = stabilizer.update({"left": fast}, 1.033)["left"]
        self.assertEqual(accepted.tracking_status, "live")
        self.assertGreater(accepted.wrist[0], 0.47)

    def test_camera_facing_short_segments_stay_live_but_do_not_poison_reference(self):
        # 与 openarm_studio 相同：前伸透视缩短的短前臂仍可用于实时控制，
        # 但不得建立/更新臂长参考。
        stabilizer = MODULE.ArmObservationStabilizer({
            "tracking": {
                "min_arm_segment_m": 0.035,
                "reference_min_arm_segment_m": 0.12,
                "segment_ratio_min": 0.15,
            }
        })
        stabilizer.update({"left": observation()}, 1.0)
        reference = stabilizer.segment_reference["left"].copy()
        foreshortened = observation(elbow=(0.0, 0.35, 0.9), wrist=(0.0, 0.40, 0.9))
        result = stabilizer.update({"left": foreshortened}, 1.05)["left"]
        self.assertEqual(result.tracking_status, "live")
        np.testing.assert_allclose(stabilizer.segment_reference["left"], reference)


class RgbArmExtractionTest(unittest.TestCase):
    def test_world_landmarks_are_extracted_without_depth_frame(self):
        class Landmark:
            def __init__(self, x=0.0, y=0.0, z=0.0, visibility=0.9):
                self.x, self.y, self.z, self.visibility = x, y, z, visibility

        landmarks = [Landmark() for _ in range(33)]
        world = [Landmark() for _ in range(33)]
        for index, point in ((11, (-0.2, 0.0, 0.0)), (13, (-0.3, 0.2, -0.1)), (15, (-0.4, 0.4, -0.2))):
            world[index] = Landmark(*point)
        result = type("Result", (), {})()
        result.pose_landmarks = type("Landmarks", (), {"landmark": landmarks})()
        result.pose_world_landmarks = type("Landmarks", (), {"landmark": world})()
        arms = MODULE.extract_rgb_arms(result, 0.55)
        self.assertIn("left", arms)
        np.testing.assert_allclose(arms["left"].wrist, [-0.4, 0.4, -0.2])


if __name__ == "__main__":
    unittest.main()

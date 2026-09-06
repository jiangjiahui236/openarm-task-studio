import importlib.util
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "openarm_task_studio" / "runtime.py"
SPEC = importlib.util.spec_from_file_location("runtime_test", MODULE_PATH)
RUNTIME = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNTIME)


class RuntimeControlHandoffTest(unittest.TestCase):
    def test_manual_gripper_request_updates_state_and_generation(self):
        before = RUNTIME.manual_gripper_generation
        RUNTIME.request_manual_gripper("closed")
        self.assertEqual(RUNTIME.manual_gripper_state, "closed")
        self.assertEqual(RUNTIME.manual_gripper_generation, before + 1)
        with self.assertRaises(ValueError):
            RUNTIME.request_manual_gripper("invalid")

    def setUp(self):
        RUNTIME.preview_request = None
        RUNTIME.preview_generation = 0
        RUNTIME.command_kind = "idle"
        RUNTIME.d435_teaching_enabled = False
        RUNTIME.d435_tracking_valid.update(left=False, right=False)

    def test_stop_motion_teaching_releases_control_for_execution(self):
        RUNTIME.d435_teaching_enabled = True
        RUNTIME.d435_tracking_valid.update(left=True, right=True)
        RUNTIME.request_preview({"position": [0.1, 0.2, 0.3]})

        RUNTIME.stop_motion_teaching()

        self.assertFalse(RUNTIME.d435_teaching_enabled)
        self.assertEqual(RUNTIME.d435_tracking_valid, {"left": False, "right": False})
        self.assertEqual(RUNTIME.command_kind, "idle")
        self.assertIsNone(RUNTIME.preview_request)

        RUNTIME.request_execution({"left": [{"position": [0.1, 0.2, 0.3]}]})
        self.assertEqual(RUNTIME.command_kind, "execute")
        self.assertIn("arm_waypoints", RUNTIME.preview_request)

    def test_recorded_joint_trajectory_skips_initial_cartesian_transit(self):
        self.assertFalse(RUNTIME.execution_needs_initial_transit([
            {"control_mode": "joint", "joint_positions": [0.1] * 7},
        ]))
        self.assertTrue(RUNTIME.execution_needs_initial_transit([
            {"control_mode": "position", "position": [0.1, 0.2, 0.3]},
        ]))


if __name__ == "__main__":
    unittest.main()

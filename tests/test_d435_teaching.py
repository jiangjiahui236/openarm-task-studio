import importlib.util
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "openarm_task_studio" / "d435_teaching.py"
SPEC = importlib.util.spec_from_file_location("d435_teaching_test", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

TUNER_PATH = Path(__file__).parents[1] / "d435_tuning_gui.py"
TUNER_SPEC = importlib.util.spec_from_file_location("task_studio_tuner_test", TUNER_PATH)
TUNER = importlib.util.module_from_spec(TUNER_SPEC)
TUNER_SPEC.loader.exec_module(TUNER)


class D435TeachingMapperTest(unittest.TestCase):
    def test_isolated_joint_target_spike_is_rejected(self):
        receiver = object.__new__(MODULE.D435TeachingReceiver)
        receiver.config = {"teaching_protection": {}}
        receiver.targets = {"left": [0.0] * 7, "right": None}
        receiver.pending_jump_targets = {"left": None, "right": None}
        receiver.filtered_spike_count = 0
        self.assertFalse(receiver._accept_mapped_target("left", [0.8] + [0.0] * 6))
        self.assertEqual(receiver.targets["left"], [0.0] * 7)
        self.assertTrue(receiver._accept_mapped_target("left", [0.02] + [0.0] * 6))
        self.assertEqual(receiver.filtered_spike_count, 1)

    def test_confirmed_large_motion_is_accepted_on_second_packet(self):
        receiver = object.__new__(MODULE.D435TeachingReceiver)
        receiver.config = {"teaching_protection": {}}
        receiver.targets = {"left": [0.0] * 7, "right": None}
        receiver.pending_jump_targets = {"left": None, "right": None}
        receiver.filtered_spike_count = 0
        self.assertFalse(receiver._accept_mapped_target("left", [0.8] + [0.0] * 6))
        self.assertTrue(receiver._accept_mapped_target("left", [0.82] + [0.0] * 6))
        self.assertAlmostEqual(receiver.targets["left"][0], 0.82)

    def test_continuous_fast_motion_is_not_filtered_indefinitely(self):
        receiver = object.__new__(MODULE.D435TeachingReceiver)
        receiver.config = {
            "teaching_protection": {
                "max_single_packet_jump_rad": 0.30,
                "jump_confirmation_tolerance_rad": 0.55,
            }
        }
        receiver.targets = {"left": [0.0] * 7, "right": None}
        receiver.pending_jump_targets = {"left": None, "right": None}
        receiver.filtered_spike_count = 0
        self.assertFalse(receiver._accept_mapped_target("left", [0.50] + [0.0] * 6))
        self.assertTrue(receiver._accept_mapped_target("left", [1.00] + [0.0] * 6))
        self.assertAlmostEqual(receiver.targets["left"][0], 1.00)

    def test_joint_speed_configuration_has_seven_safe_values(self):
        receiver = object.__new__(MODULE.D435TeachingReceiver)
        receiver.config = {"teaching_protection": {"max_joint_speed_rad_s": [99.0] * 7}}
        self.assertEqual(receiver.max_joint_speeds(), [20.0] * 7)

    def test_velocity_feedforward_gain_is_configurable_and_bounded(self):
        receiver = object.__new__(MODULE.D435TeachingReceiver)
        receiver.config = {"control": {"velocity_feedforward_gain": 1.5}}
        self.assertEqual(receiver.velocity_feedforward_gain(), 1.5)
        receiver.config["control"]["velocity_feedforward_gain"] = 99.0
        self.assertEqual(receiver.velocity_feedforward_gain(), 2.0)

    def test_task_studio_tuner_allows_model_elbow_limit(self):
        upstream = TUNER.load_upstream_tuner()
        specs = TUNER.task_studio_specs(upstream)
        spec = next(
            item for item in specs
            if item.label == "肘部最大弯曲"
        )
        self.assertAlmostEqual(spec.maximum, 2.443)
        self.assertEqual(spec.affected_motors, "J4")
        self.assertTrue(all(item.affected_motors for item in specs))

    def test_default_config_is_owned_by_task_studio(self):
        project_dir = MODULE_PATH.parents[1]
        self.assertEqual(
            MODULE.DEFAULT_CONFIG,
            project_dir / "config" / "openarm_d435_teleop_visual.yaml",
        )
        self.assertTrue(MODULE.DEFAULT_CONFIG.is_file())

    def test_neutral_observation_produces_seven_finite_joint_targets(self):
        receiver = object.__new__(MODULE.D435TeachingReceiver)
        with MODULE.DEFAULT_CONFIG.open("r", encoding="utf-8") as stream:
            receiver.config = MODULE.yaml.safe_load(stream)
        observation = {
            "target_position": [-0.08, 0.22, 0.32],
            "shoulder_camera": [0.0, 0.0, 1.0],
            "elbow_camera": [0.1, 0.1, 1.0],
            "wrist_camera": [0.2, 0.1, 1.0],
        }
        targets = receiver._map("left", observation)
        self.assertEqual(len(targets), 7)
        self.assertTrue(all(MODULE.math.isfinite(value) for value in targets))
        self.assertEqual(len(receiver.neutral_targets("left")), 7)

        rotated = dict(observation)
        rotated["wrist_camera"] = [0.2, 0.1, 1.2]
        rotated_targets = receiver._map("left", rotated)
        self.assertNotAlmostEqual(rotated_targets[2], targets[2])

        right_angle = dict(observation)
        right_angle["wrist_camera"] = [0.2, 0.0, 1.0]
        right_angle_targets = receiver._map("left", right_angle)
        joint4_config = receiver.config["arms"]["left"]["joint_pose"]
        elbow_amount = (
            joint4_config["elbow_straight_deg"] - 90.0
        ) / (
            joint4_config["elbow_straight_deg"] - joint4_config["elbow_bent_deg"]
        )
        expected_joint4 = min(
            joint4_config["joint4_max"],
            joint4_config["joint4_base"]
            + joint4_config["joint4_elbow_bend_gain"] * elbow_amount,
        )
        self.assertAlmostEqual(right_angle_targets[3], expected_joint4, places=2)

        right_neutral = dict(observation)
        right_neutral["target_position"] = [-0.08, -0.22, 0.32]
        receiver._map("right", right_neutral)
        right_rotated = dict(rotated)
        right_rotated["target_position"] = [-0.08, -0.22, 0.32]
        right_targets = receiver._map("right", right_rotated)
        self.assertNotAlmostEqual(rotated_targets[2], targets[2])
        self.assertNotAlmostEqual(right_targets[2], 0.0)

    def test_missing_tracking_has_infinite_target_age(self):
        receiver = object.__new__(MODULE.D435TeachingReceiver)
        receiver.last_valid_time = {"left": None, "right": None}
        self.assertTrue(MODULE.math.isinf(receiver.target_age("left")))

    def test_wrist_moving_backward_drives_mirrored_joint1(self):
        receiver = object.__new__(MODULE.D435TeachingReceiver)
        with MODULE.DEFAULT_CONFIG.open("r", encoding="utf-8") as stream:
            receiver.config = MODULE.yaml.safe_load(stream)
        observation = {
            "shoulder_camera": [0.0, 0.0, 1.0],
            "elbow_camera": [0.1, 0.1, 1.0],
            "wrist_camera": [0.2, 0.1, 1.0],
        }
        for arm, expected_sign in (("left", 1.0), ("right", -1.0)):
            arm_config = receiver.config["arms"][arm]
            neutral_observation = dict(observation, target_position=list(arm_config["neutral_position"]))
            backward_observation = dict(observation, target_position=list(arm_config["neutral_position"]))
            backward_observation["target_position"][0] = arm_config["workspace_min"][0]
            neutral_joint1 = receiver._map(arm, neutral_observation)[0]
            backward_joint1 = receiver._map(arm, backward_observation)[0]
            configured_gain = abs(float(arm_config["joint_pose"]["joint1_backward_gain"]))
            self.assertGreater(
                expected_sign * (backward_joint1 - neutral_joint1),
                0.95 * configured_gain,
            )

    def test_task_studio_tuner_controls_backward_gain_and_limit(self):
        upstream = TUNER.load_upstream_tuner()
        specs = TUNER.task_studio_specs(upstream)
        by_label = {spec.label: spec for spec in specs}
        self.assertIn("上臂后摆增益", by_label)
        self.assertIn("上臂后摆上限", by_label)
        self.assertIn("上臂后收满值", by_label)
        self.assertEqual(by_label["上臂后摆增益"].maximum, 3.0)
        self.assertEqual(by_label["上臂后摆上限"].maximum, 2.0)
        with MODULE.DEFAULT_CONFIG.open("r", encoding="utf-8") as stream:
            config = MODULE.yaml.safe_load(stream)
        by_label["上臂后摆增益"].setter(config, 0.9)
        by_label["上臂后摆上限"].setter(config, 1.0)
        self.assertEqual(config["arms"]["left"]["joint_pose"]["joint1_backward_gain"], 0.9)
        self.assertEqual(config["arms"]["right"]["joint_pose"]["joint1_backward_gain"], -0.9)
        self.assertEqual(config["arms"]["left"]["joint_pose"]["joint1_max"], 1.0)
        self.assertEqual(config["arms"]["right"]["joint_pose"]["joint1_min"], -1.0)

    def test_upper_arm_retraction_drives_joint1_while_wrist_target_stays_neutral(self):
        receiver = object.__new__(MODULE.D435TeachingReceiver)
        with MODULE.DEFAULT_CONFIG.open("r", encoding="utf-8") as stream:
            receiver.config = MODULE.yaml.safe_load(stream)
        receiver.upper_arm_backward_reference = {"left": None, "right": None}
        for arm, expected_sign in (("left", 1.0), ("right", -1.0)):
            target = list(receiver.config["arms"][arm]["neutral_position"])
            neutral = {
                "target_position": target,
                "shoulder_camera": [0.0, 0.0, 1.0],
                "elbow_camera": [0.0, 0.25, 1.0],
                "wrist_camera": [0.0, 0.10, 1.15],
            }
            retracted = dict(
                neutral,
                elbow_camera=[0.0, 0.15, 1.25],
                wrist_camera=[0.0, 0.05, 1.10],
            )
            neutral_joint1 = receiver._map(arm, neutral)[0]
            retracted_joint1 = receiver._map(arm, retracted)[0]
            configured_gain = abs(float(receiver.config["arms"][arm]["joint_pose"]["joint1_backward_gain"]))
            self.assertGreater(
                expected_sign * (retracted_joint1 - neutral_joint1),
                0.95 * configured_gain,
            )

    def test_tuner_control_and_joint4_parameters_drive_current_mapping(self):
        receiver = object.__new__(MODULE.D435TeachingReceiver)
        with MODULE.DEFAULT_CONFIG.open("r", encoding="utf-8") as stream:
            receiver.config = MODULE.yaml.safe_load(stream)
        self.assertEqual(receiver.command_rate_hz(), receiver.config["control"]["command_rate_hz"])
        self.assertEqual(receiver.trajectory_time_s(), receiver.config["control"]["trajectory_time_s"])

        observation = {
            "target_position": [-0.08, 0.22, 0.50],
            "shoulder_camera": [0.0, 0.0, 1.0],
            "elbow_camera": [0.1, 0.1, 1.0],
            "wrist_camera": [0.2, 0.0, 1.0],
        }
        baseline = receiver._map("left", observation)[3]
        config = receiver.config["arms"]["left"]["joint_pose"]
        config["joint4_elbow_bend_gain"] = 0.4
        config["joint4_up_gain"] = 0.6
        changed = receiver._map("left", observation)[3]
        self.assertNotAlmostEqual(changed, baseline)


if __name__ == "__main__":
    unittest.main()

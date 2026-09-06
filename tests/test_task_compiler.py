import importlib.util
import sys
import types
import unittest
from pathlib import Path


PACKAGE_DIR = Path(__file__).parents[1] / "openarm_task_studio"
PACKAGE_NAME = "openarm_task_compiler_test"
package = types.ModuleType(PACKAGE_NAME)
package.__path__ = [str(PACKAGE_DIR)]
sys.modules[PACKAGE_NAME] = package


def load_module(name):
    spec = importlib.util.spec_from_file_location(
        f"{PACKAGE_NAME}.{name}", PACKAGE_DIR / f"{name}.py"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


MODEL = load_module("task_model")
COMPILER = load_module("task_compiler")
TaskNode = MODEL.TaskNode
CompilerConfig = COMPILER.CompilerConfig
TaskCompiler = COMPILER.TaskCompiler


class TaskCompilerTest(unittest.TestCase):
    def compiler(self, blocked=False):
        return TaskCompiler(
            CompilerConfig(table_height=0.35, max_cartesian_step=0.06),
            segment_blocked=lambda _start, _end, _margin: blocked,
            safe_transit_height=lambda: 0.60,
            joint_limits=lambda _arm: [[-3.14, 3.14]] * 7,
        )

    @staticmethod
    def node(task_type, name, position, arm="left", **parameters):
        return TaskNode(
            node_id=f"id-{name}", task_type=task_type, name=name,
            prim_path=f"/World/{name}", arm=arm,
            parameters={"position": position, **parameters},
        )

    def test_compile_grasp_has_explicit_stages_and_shared_orientation(self):
        nodes = [
            self.node("move", "move1", [0.30, 0.0, 0.45]),
            self.node("grasp", "grasp1", [0.40, 0.0, 0.36], grasp_orientation="horizontal"),
        ]
        waypoints = self.compiler().compile_arm(nodes, "left", [0.0] * 7, 1.5)
        phases = [item["phase"] for item in waypoints]
        self.assertIn("early-prealign", phases)
        self.assertIn("prealign", phases)
        self.assertIn("align", phases)
        self.assertIn("grasp", phases)
        grasp_stages = [item for item in waypoints if item.get("node_name") == "grasp1"]
        orientation_groups = {
            item["orientation_group"] for item in grasp_stages if item.get("level_gripper")
        }
        self.assertEqual(len(orientation_groups), 1)
        early = next(item for item in grasp_stages if item["phase"] == "early-prealign")
        self.assertFalse(early.get("level_gripper", False))
        approach = next(item for item in grasp_stages if item["phase"] == "approach")
        self.assertFalse(approach.get("level_gripper", False))
        self.assertEqual(approach["tolerance"], 0.05)
        self.assertEqual(approach["orientation_tolerance"], 0.12)
        self.assertEqual(approach["max_retries"], 2)
        retreat = next(item for item in grasp_stages if item["phase"] == "retreat")
        self.assertFalse(retreat.get("level_gripper", False))
        self.assertEqual(retreat["tolerance"], 0.04)
        self.assertEqual(retreat["max_retries"], 2)
        prealign = next(item for item in grasp_stages if item["phase"] == "prealign")
        self.assertEqual(prealign["tolerance"], 0.04)
        self.assertEqual(prealign["orientation_tolerance"], 0.12)
        self.assertEqual(prealign["max_retries"], 2)
        self.assertTrue(all(
            item["approach_direction"] == [0.0, 0.0, -1.0]
            for item in grasp_stages
        ))
        self.assertTrue(all(item["speed_scale"] == 1.5 for item in waypoints))
        self.assertTrue(all("stage_id" in item for item in waypoints))

    def test_action_height_respects_offset_and_table_clearance(self):
        compiler = self.compiler()
        position = compiler.action_tcp_position(
            [0.4, 0.0, 0.36],
            {"grasp_height_offset": 0.02, "minimum_wrist_clearance": 0.08},
        )
        self.assertEqual(position, [0.4, 0.0, 0.43])

    def test_recorded_grasp_uses_joint_pose_without_forced_prealign(self):
        node = self.node(
            "grasp", "grasp1", [0.4, 0.0, 0.36],
            grasp_orientation="vertical", joint_positions=[0.1] * 7,
        )
        waypoints = self.compiler().compile_arm([node], "left")
        approach = next(item for item in waypoints if item["phase"] == "approach")
        prealign = next(item for item in waypoints if item["phase"] == "prealign")
        align = next(item for item in waypoints if item["phase"] == "align")
        self.assertFalse(approach.get("level_gripper", False))
        self.assertFalse(prealign.get("level_gripper", False))
        self.assertEqual(align["joint_positions"], [0.1] * 7)
        self.assertEqual(align["joint_tolerance"], 0.20)

    def test_obstacle_transit_lifts_and_subdivides(self):
        nodes = [
            self.node("move", "move1", [0.20, 0.0, 0.40]),
            self.node("move", "move2", [0.50, 0.0, 0.40]),
        ]
        waypoints = self.compiler(blocked=True).compile_arm(nodes, "left")
        phases = [item["phase"] for item in waypoints]
        self.assertIn("move-lift", phases)
        self.assertIn("move-transit", phases)
        self.assertTrue(any("step" in phase for phase in phases))

    def test_right_horizontal_wrist_is_mirrored(self):
        node = self.node(
            "grasp", "grasp1", [0.4, -0.1, 0.36], arm="right",
            grasp_orientation="horizontal",
        )
        waypoints = self.compiler().compile_preview(node, [0.0] * 7)
        wrist_angles = {item["wrist_angle"] for item in waypoints if "wrist_angle" in item}
        self.assertEqual(wrist_angles, {-1.5707963267948966})

    def test_move_joint_override_is_joint_control(self):
        node = self.node(
            "move", "move1", [0.4, 0.0, 0.45], joint_positions=[0.1] * 7,
        )
        waypoint = self.compiler().compile_preview(node, [0.0] * 7)[-1]
        self.assertEqual(waypoint["control_mode"], "joint")
        self.assertEqual(waypoint["joint_positions"], [0.1] * 7)
        self.assertEqual(waypoint["joint_tolerance"], 0.15)
        self.assertNotIn("tolerance", waypoint)

    def test_recorded_joint_moves_do_not_gain_cartesian_interpolation(self):
        nodes = [
            self.node("move", "move1", [0.20, 0.0, 0.15], joint_positions=[0.1] * 7),
            self.node("move", "move2", [0.50, 0.0, 0.45], joint_positions=[0.2] * 7),
        ]

        waypoints = self.compiler(blocked=True).compile_arm(nodes, "left")

        self.assertEqual(len(waypoints), 2)
        self.assertEqual([item["node_name"] for item in waypoints], ["move1", "move2"])
        self.assertTrue(all(item["control_mode"] == "joint" for item in waypoints))


if __name__ == "__main__":
    unittest.main()

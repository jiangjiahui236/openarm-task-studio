import json
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path

MODULE_PATH = Path(__file__).parents[1] / "openarm_task_studio" / "task_model.py"
SPEC = importlib.util.spec_from_file_location("openarm_task_model_test", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
TaskDocument, TaskNode = MODULE.TaskDocument, MODULE.TaskNode


class TaskModelTest(unittest.TestCase):
    def test_saved_document_has_execution_boundaries(self):
        document = TaskDocument()
        document.nodes.append(TaskNode("node-1", "move", "Move 1", "/World/Move_1", {"position": [0, 0, 0]}))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "task.json"
            document.save(path)
            data = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(data["schema"], "openarm.task-studio/v3")
        self.assertEqual(data["scene_constraints"]["collision_geometry"], "usd_scene_obstacles")
        self.assertEqual(data["execution"]["workspace_validation"], "replaced_by_scene_obstacle_validation")
        self.assertEqual(data["execution"]["robot_ik"], "simulation_preview_position_only")

    def test_document_stores_planning_defaults(self):
        document = TaskDocument(collision_margin=0.04, default_approach_height=0.12, execution_speed=1.75)
        data = document.to_dict()
        self.assertEqual(data["scene_constraints"]["collision_margin"], 0.04)
        self.assertEqual(data["scene_constraints"]["default_approach_height"], 0.12)
        self.assertEqual(data["execution"]["speed_scale"], 1.75)
        self.assertEqual(data["scene_constraints"]["front_workbench_base_z"], 0.0)
        self.assertEqual(data["scene_constraints"]["robot_base_z"], 0.0)
        self.assertEqual(data["scene_constraints"]["front_workbench_top_height"], 0.35)

    def test_document_round_trips_front_workbench_configuration(self):
        document = TaskDocument(
            front_workbench_enabled=False,
            front_workbench_top_height=1.05,
            front_workbench_distance=0.35,
            front_workbench_base_z=0.61,
            robot_base_z=0.0,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "task.json"
            document.save(path)
            loaded = TaskDocument.load(path)
        self.assertFalse(loaded.front_workbench_enabled)
        self.assertEqual(loaded.front_workbench_top_height, 1.05)
        self.assertEqual(loaded.front_workbench_distance, 0.35)
        self.assertEqual(loaded.front_workbench_base_z, 0.61)
        self.assertEqual(loaded.robot_base_z, 0.0)

    def test_document_round_trips_execution_speed_and_defaults_legacy(self):
        document = TaskDocument(execution_speed=0.6)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "task.json"
            document.save(path)
            loaded = TaskDocument.load(path)
            data = json.loads(path.read_text(encoding="utf-8"))
            data["execution"].pop("speed_scale")
            path.write_text(json.dumps(data), encoding="utf-8")
            legacy_loaded = TaskDocument.load(path)
        self.assertEqual(loaded.execution_speed, 0.6)
        self.assertEqual(legacy_loaded.execution_speed, 3.0)

    def test_legacy_document_marks_workbench_height_for_migration(self):
        document = TaskDocument().to_dict()
        constraints = document["scene_constraints"]
        constraints.pop("front_workbench_top_height")
        constraints.pop("front_workbench_enabled")
        constraints.pop("front_workbench_distance")
        constraints.pop("robot_base_z")
        constraints["front_workbench_base_z"] = 0.75
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "legacy-task.json"
            path.write_text(json.dumps(document), encoding="utf-8")
            loaded = TaskDocument.load(path)
        self.assertIsNone(loaded.front_workbench_top_height)
        self.assertTrue(loaded.front_workbench_enabled)
        self.assertEqual(loaded.front_workbench_distance, 0.20)
        self.assertEqual(loaded.front_workbench_base_z, 0.75)
        self.assertEqual(loaded.robot_base_z, 0.75)

    def test_saved_document_can_be_loaded_with_joint_positions(self):
        document = TaskDocument()
        document.nodes.append(TaskNode(
            "node-1", "move", "move1", "/World/Move_1",
            {"position": [0.4, 0.0, 1.3], "joint_positions": [0.1] * 7},
        ))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "task.json"
            document.save(path)
            loaded = TaskDocument.load(path)
        self.assertEqual(loaded.nodes[0].name, "move1")
        self.assertEqual(loaded.nodes[0].parameters["joint_positions"], [0.1] * 7)

    def test_nodes_round_trip_arm_assignment_and_default_left(self):
        document = TaskDocument(nodes=[
            TaskNode("right-1", "move", "move1", "/World/Move_1", {"position": [0, 0, 0]}, arm="right"),
        ])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "task.json"
            document.save(path)
            loaded = TaskDocument.load(path)
            data = json.loads(path.read_text(encoding="utf-8"))
            data["nodes"][0].pop("arm")
            path.write_text(json.dumps(data), encoding="utf-8")
            legacy_loaded = TaskDocument.load(path)
        self.assertEqual(loaded.nodes[0].arm, "right")
        self.assertEqual(legacy_loaded.nodes[0].arm, "left")


if __name__ == "__main__":
    unittest.main()

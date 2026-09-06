from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any


@dataclass
class TaskNode:
    node_id: str
    task_type: str
    name: str
    prim_path: str
    parameters: dict[str, Any]
    confirmed: bool = True
    arm: str = "left"


@dataclass
class TaskDocument:
    name: str = "OpenArm Production Task"
    nodes: list[TaskNode] = field(default_factory=list)
    collision_margin: float = 0.02
    default_approach_height: float = 0.06
    execution_speed: float = 3.0
    robot_base_z: float = 0.0
    front_workbench_base_z: float = 0.0
    front_workbench_enabled: bool = True
    front_workbench_top_height: float | None = 0.35
    front_workbench_distance: float = 0.20

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": "openarm.task-studio/v3",
            "name": self.name,
            "updated_at": datetime.now(timezone.utc).isoformat(),
            "scene_constraints": {
                "collision_geometry": "usd_scene_obstacles",
                "collision_margin": self.collision_margin,
                "default_approach_height": self.default_approach_height,
                "robot_base_z": self.robot_base_z,
                "front_workbench_base_z": self.front_workbench_base_z,
                "front_workbench_enabled": self.front_workbench_enabled,
                "front_workbench_top_height": self.front_workbench_top_height,
                "front_workbench_distance": self.front_workbench_distance,
            },
            "nodes": [asdict(node) for node in self.nodes],
            "execution": {
                "speed_scale": self.execution_speed,
                "workspace_validation": "replaced_by_scene_obstacle_validation",
                "tcp_scene_collision_sampling": "implemented",
                "robot_ik": "simulation_preview_position_only",
                "swept_collision_check": "not_connected",
            },
        }

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.to_dict(), indent=2), encoding="utf-8")

    @classmethod
    def load(cls, path: Path) -> "TaskDocument":
        data = json.loads(path.read_text(encoding="utf-8"))
        if data.get("schema") != "openarm.task-studio/v3":
            raise ValueError(f"Unsupported task schema: {data.get('schema')}")
        constraints = data.get("scene_constraints", {})
        execution = data.get("execution", {})
        return cls(
            name=data.get("name", "OpenArm Production Task"),
            nodes=[TaskNode(**node) for node in data.get("nodes", [])],
            collision_margin=float(constraints.get("collision_margin", 0.02)),
            default_approach_height=float(constraints.get("default_approach_height", 0.06)),
            execution_speed=float(execution.get("speed_scale", 3.0)),
            robot_base_z=float(constraints.get("robot_base_z", 0.75)),
            front_workbench_base_z=float(constraints.get("front_workbench_base_z", 0.75)),
            front_workbench_enabled=bool(constraints.get("front_workbench_enabled", True)),
            front_workbench_top_height=(
                float(constraints["front_workbench_top_height"])
                if "front_workbench_top_height" in constraints else None
            ),
            front_workbench_distance=float(constraints.get("front_workbench_distance", 0.20)),
        )

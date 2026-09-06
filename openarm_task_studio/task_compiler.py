from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Iterable

from .task_model import TaskNode


Position = list[float]
Waypoint = dict[str, object]


@dataclass(frozen=True)
class CompilerConfig:
    collision_margin: float = 0.02
    default_approach_height: float = 0.06
    max_cartesian_step: float = 0.06
    table_height: float = 0.35
    horizontal_grasp_angles: tuple[float, float] = (math.pi / 2.0, -math.pi / 2.0)


class TaskCompiler:
    def __init__(
        self,
        config: CompilerConfig,
        segment_blocked: Callable[[Position, Position, float], bool],
        safe_transit_height: Callable[[], float],
        joint_limits: Callable[[str], list[list[float]] | None],
    ):
        self.config = config
        self.segment_blocked = segment_blocked
        self.safe_transit_height = safe_transit_height
        self.joint_limits = joint_limits

    def compile_arm(
        self,
        nodes: Iterable[TaskNode],
        arm: str,
        current_joints: list[float] | None = None,
        speed_scale: float = 1.0,
    ) -> list[Waypoint]:
        waypoints: list[Waypoint] = []
        wrist_reference = float((current_joints or [0.0] * 7)[-1])
        for node in nodes:
            node_start = len(waypoints)
            object_target = [float(value) for value in node.parameters["position"]]
            joints = node.parameters.get("joint_positions")
            if node.task_type == "move":
                if joints:
                    # Motion teaching records a realizable joint-space sample. Replaying
                    # Cartesian interpolation between samples changes that trajectory and
                    # can manufacture TCP/table collisions that were never recorded.
                    waypoints.append(self._waypoint(
                        object_target, "hold", node, "move", joint_positions=joints,
                    ))
                    wrist_reference = float(joints[-1])
                else:
                    self._append_transit(waypoints, object_target, "hold", node, "move")
                    waypoints[-1]["hold_time"] = 0.05
                self._finalize_node(waypoints[node_start:], node, arm, speed_scale)
                continue

            target = self.action_tcp_position(object_target, node.parameters)
            approach_height = float(
                node.parameters.get("approach_height", self.config.default_approach_height)
            )
            above = [target[0], target[1], target[2] + approach_height]
            base_wrist = self._grasp_wrist_angle(node)
            wrist_target = (
                float(joints[-1])
                if joints else self._nearest_equivalent_wrist(base_wrist, wrist_reference, arm)
            )
            wrist_reference = wrist_target
            orientation_group = f"{arm}:{node.node_id}:grasp-frame"

            if node.task_type == "grasp":
                if waypoints:
                    early = self._waypoint(
                        list(waypoints[-1]["position"]), "open", node, "early-prealign",
                        wrist_angle=wrist_target,
                    )
                    early.update({"tolerance": 0.045, "orientation_tolerance": 0.08})
                    waypoints.append(early)
                self._append_transit(
                    waypoints, above, "open", node, "approach",
                    wrist_angle=wrist_target,
                )
                waypoints.extend([
                    self._waypoint(
                        above, "open", node, "prealign", wrist_angle=wrist_target,
                        lock_orientation=not bool(joints),
                        orientation_group=orientation_group if not joints else None,
                    ),
                    self._waypoint(
                        target, "open", node, "align", wrist_angle=wrist_target,
                        joint_positions=joints, lock_orientation=not bool(joints),
                        orientation_group=orientation_group if not joints else None,
                    ),
                    self._waypoint(
                        target, "closed", node, "grasp", action="attach",
                        object_path=node.prim_path, wrist_angle=wrist_target,
                        object_parameters=node.parameters, joint_positions=joints,
                        lock_orientation=not bool(joints),
                        orientation_group=orientation_group if not joints else None,
                    ),
                    self._waypoint(
                        above, "closed", node, "retreat", wrist_angle=wrist_target,
                    ),
                ])
            else:
                self._append_transit(waypoints, above, "closed", node, "approach")
                waypoints.extend([
                    self._waypoint(
                        above, "closed", node, "prealign", wrist_angle=wrist_target,
                        lock_orientation=not bool(joints),
                        orientation_group=orientation_group if not joints else None,
                    ),
                    self._waypoint(
                        target, "closed", node, "align", wrist_angle=wrist_target,
                        joint_positions=joints, lock_orientation=not bool(joints),
                        orientation_group=orientation_group if not joints else None,
                    ),
                    self._waypoint(
                        target, "open", node, "release", action="detach",
                        object_path=node.prim_path, wrist_angle=wrist_target,
                        joint_positions=joints, lock_orientation=not bool(joints),
                        orientation_group=orientation_group if not joints else None,
                    ),
                    self._waypoint(
                        above, "open", node, "retreat", wrist_angle=wrist_target,
                    ),
                ])

            for waypoint in waypoints[node_start:]:
                if waypoint.get("action") in ("attach", "detach"):
                    waypoint["object_position"] = object_target
                    waypoint["attach_distance_limit"] = max(
                        0.04, abs(target[2] - object_target[2]) + 0.015
                    )
            self._finalize_node(waypoints[node_start:], node, arm, speed_scale)
        return waypoints

    def compile_preview(
        self,
        node: TaskNode,
        current_joints: list[float] | None = None,
        speed_scale: float = 1.0,
    ) -> list[Waypoint]:
        return self.compile_arm([node], node.arm, current_joints, speed_scale)

    def action_tcp_position(self, object_position, parameters) -> Position:
        height_offset = max(0.0, float(parameters.get("grasp_height_offset", 0.0)))
        wrist_clearance = max(0.0, float(parameters.get("minimum_wrist_clearance", 0.0)))
        return [
            float(object_position[0]),
            float(object_position[1]),
            max(
                float(object_position[2]) + height_offset,
                self.config.table_height + wrist_clearance,
            ),
        ]

    def planned_positions(self, nodes: Iterable[TaskNode], arm: str) -> list[Position]:
        return [
            list(waypoint["position"])
            for waypoint in self.compile_arm(nodes, arm, speed_scale=1.0)
        ]

    def _finalize_node(
        self, waypoints: list[Waypoint], node: TaskNode, arm: str, speed_scale: float
    ) -> None:
        for index, waypoint in enumerate(waypoints):
            waypoint["arm"] = arm
            waypoint["node_id"] = node.node_id
            waypoint["node_name"] = node.name
            waypoint["task_type"] = node.task_type
            waypoint["stage_index"] = index
            waypoint["stage_id"] = f"{arm}:{node.node_id}:{waypoint['phase']}:{index}"
            waypoint["speed_scale"] = max(0.1, min(3.0, float(speed_scale)))
            if node.task_type in ("grasp", "place"):
                waypoint["approach_direction"] = list(
                    node.parameters.get("approach_direction", [0.0, 0.0, -1.0])
                )
            if waypoint["phase"] == "approach":
                waypoint["tolerance"] = 0.05
                waypoint["orientation_tolerance"] = 0.12
            if waypoint["phase"] == "prealign":
                waypoint["tolerance"] = 0.04
                waypoint["orientation_tolerance"] = 0.12
            if waypoint["phase"] == "retreat":
                waypoint["tolerance"] = 0.04
            if (
                node.task_type in ("grasp", "place")
                and waypoint.get("joint_positions")
            ):
                waypoint["joint_tolerance"] = 0.20
            if node.task_type == "move" and waypoint.get("joint_positions"):
                waypoint["joint_tolerance"] = 0.15
            waypoint.setdefault("failure_policy", "abort_arm")
            waypoint.setdefault(
                "max_retries",
                2 if waypoint["phase"] in ("approach", "prealign", "retreat") else 0,
            )

    def _waypoint(
        self,
        position,
        gripper,
        node,
        phase,
        action=None,
        object_path=None,
        wrist_angle=None,
        object_parameters=None,
        joint_positions=None,
        lock_orientation=False,
        orientation_group=None,
    ) -> Waypoint:
        display_phase = {
            "early-prealign": "early pre-align",
            "prealign": "pre-align",
            "grasp": "close",
        }.get(phase, phase.replace("-", " "))
        waypoint: Waypoint = {
            "position": list(position),
            "gripper": gripper,
            "phase": phase,
            "label": f"{node.name} {display_phase}",
            "control_mode": "position",
        }
        if action:
            waypoint["action"] = action
        if object_path:
            waypoint["object_path"] = object_path
        if wrist_angle is not None:
            waypoint["wrist_angle"] = float(wrist_angle)
        if lock_orientation:
            waypoint["lock_wrist"] = True
            waypoint["level_gripper"] = True
            waypoint["control_mode"] = "level_pose"
        if orientation_group:
            waypoint["orientation_group"] = orientation_group
        if object_parameters:
            waypoint["shape"] = object_parameters.get("shape", "box")
            waypoint["dimensions"] = list(
                object_parameters.get("dimensions", [0.03, 0.03, 0.03])
            )
        if joint_positions:
            waypoint["joint_positions"] = list(joint_positions)
            waypoint["control_mode"] = "joint"
        return waypoint

    def _append_transit(
        self,
        waypoints,
        target,
        gripper,
        node,
        phase,
        wrist_angle=None,
        lock_orientation=False,
        orientation_group=None,
    ) -> None:
        def transit(position, transit_phase):
            waypoint = self._waypoint(
                position, gripper, node, transit_phase, wrist_angle=wrist_angle,
                lock_orientation=lock_orientation, orientation_group=orientation_group,
            )
            if lock_orientation:
                waypoint["orientation_tolerance"] = 0.08
            return waypoint

        target = list(target)
        if waypoints:
            start = list(waypoints[-1]["position"])
            if self.segment_blocked(start, target, self.config.collision_margin):
                safe_z = max(self.safe_transit_height(), start[2], target[2])
                if abs(start[2] - safe_z) > 0.005:
                    waypoints.append(transit([start[0], start[1], safe_z], f"{phase}-lift"))
                horizontal = [target[0], target[1], safe_z]
                if any(abs(horizontal[i] - waypoints[-1]["position"][i]) > 0.005 for i in range(3)):
                    waypoints.append(transit(horizontal, f"{phase}-transit"))
        if waypoints:
            start = list(waypoints[-1]["position"])
            distance = math.dist(start, target)
            segment_count = max(1, math.ceil(distance / self.config.max_cartesian_step))
            for segment in range(1, segment_count):
                ratio = segment / segment_count
                intermediate = [
                    start[axis] + (target[axis] - start[axis]) * ratio for axis in range(3)
                ]
                waypoint = transit(intermediate, f"{phase}-step-{segment}-of-{segment_count}")
                waypoint.update({"tolerance": 0.04, "hold_time": 0.0})
                waypoints.append(waypoint)
        waypoints.append(transit(target, phase))

    def _grasp_wrist_angle(self, node: TaskNode) -> float:
        if node.parameters.get("grasp_orientation") != "horizontal":
            return 0.0
        return self.config.horizontal_grasp_angles[1 if node.arm == "right" else 0]

    def _nearest_equivalent_wrist(self, base_angle, reference_angle, arm) -> float:
        limits = self.joint_limits(arm)
        lower, upper = limits[-1] if limits and len(limits) == 7 else (-math.pi, math.pi)
        candidates = [
            float(base_angle) + turn * math.pi
            for turn in range(-3, 4)
            if lower <= float(base_angle) + turn * math.pi <= upper
        ]
        if not candidates:
            return max(lower, min(upper, float(base_angle)))
        return min(candidates, key=lambda angle: abs(angle - float(reference_angle)))

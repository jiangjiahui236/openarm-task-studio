from __future__ import annotations

import json
import math
import socket
import time
from pathlib import Path

import yaml


DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config" / "openarm_d435_teleop_visual.yaml"


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


class D435TeachingReceiver:
    def __init__(self, host="127.0.0.1", port=5010, config_path=DEFAULT_CONFIG):
        self.config_path = Path(config_path)
        with self.config_path.open("r", encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream)
        self.config_mtime = self.config_path.stat().st_mtime
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((host, port))
        self.socket.setblocking(False)
        self.targets = {arm: None for arm in ("left", "right")}
        self.raw_packet_count = 0
        self.packet_count = 0
        self.last_valid_time = {arm: None for arm in ("left", "right")}
        self.upper_arm_backward_reference = {arm: None for arm in ("left", "right")}
        self.pending_jump_targets = {arm: None for arm in ("left", "right")}
        self.filtered_spike_count = 0
        self.latest_metadata = {arm: {} for arm in ("left", "right")}

    def neutral_targets(self, arm):
        return [float(value) for value in self.config["arms"][arm]["joint_pose"]["neutral_positions"]]

    def command_rate_hz(self):
        return _clamp(float(self.config.get("control", {}).get("command_rate_hz", 10.0)), 1.0, 30.0)

    def trajectory_time_s(self):
        return _clamp(float(self.config.get("control", {}).get("trajectory_time_s", 0.29)), 0.05, 5.0)

    def velocity_feedforward_gain(self):
        return _clamp(float(self.config.get("control", {}).get("velocity_feedforward_gain", 1.0)), 0.0, 2.0)

    def poll(self):
        self._reload_config()
        while True:
            try:
                data, _address = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            try:
                payload = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if payload.get("type") != "openarm_arm_pose_targets" or not payload.get("calibrated"):
                if payload.get("type") == "openarm_arm_pose_targets":
                    self.raw_packet_count += 1
                    self.upper_arm_backward_reference = {arm: None for arm in ("left", "right")}
                    self.pending_jump_targets = {arm: None for arm in ("left", "right")}
                continue
            self.raw_packet_count += 1
            self.packet_count += 1
            for arm in ("left", "right"):
                observation = payload.get("arms", {}).get(arm)
                if observation and float(observation.get("confidence", 0.0)) >= 0.45:
                    mapped = self._map(arm, observation)
                    accepted = self._accept_mapped_target(arm, mapped)
                    self.latest_metadata[arm] = {
                        "source_timestamp": payload.get("timestamp"),
                        "source_frame": payload.get("frame"),
                        "source_fps": payload.get("fps"),
                        "tracking_status": observation.get("tracking_status", "unknown"),
                        "confidence": observation.get("confidence"),
                        "shoulder_camera": observation.get("shoulder_camera"),
                        "elbow_camera": observation.get("elbow_camera"),
                        "wrist_camera": observation.get("wrist_camera"),
                        "target_position": observation.get("target_position"),
                        "accepted": accepted,
                    }
                    self.last_valid_time[arm] = time.monotonic()
        return self.targets

    def _accept_mapped_target(self, arm, mapped):
        previous = self.targets[arm]
        if previous is None:
            self.targets[arm] = list(mapped)
            self.pending_jump_targets[arm] = None
            return True
        protection = self.config.get("teaching_protection", {})
        jump_limit = float(protection.get("max_single_packet_jump_rad", 0.30))
        confirmation_tolerance = float(protection.get("jump_confirmation_tolerance_rad", 0.18))
        jump = max(abs(float(value) - float(old)) for value, old in zip(mapped, previous))
        if jump <= jump_limit:
            self.targets[arm] = list(mapped)
            self.pending_jump_targets[arm] = None
            return True
        pending = self.pending_jump_targets[arm]
        if pending is not None:
            confirmation_error = max(
                abs(float(value) - float(candidate))
                for value, candidate in zip(mapped, pending)
            )
            if confirmation_error <= confirmation_tolerance:
                self.targets[arm] = list(mapped)
                self.pending_jump_targets[arm] = None
                return True
        self.pending_jump_targets[arm] = list(mapped)
        self.filtered_spike_count += 1
        return False

    def max_joint_speeds(self):
        configured = self.config.get("teaching_protection", {}).get(
            "max_joint_speed_rad_s", [1.8, 1.8, 2.0, 2.0, 2.5, 2.5, 2.5]
        )
        if not isinstance(configured, list) or len(configured) != 7:
            configured = [1.8, 1.8, 2.0, 2.0, 2.5, 2.5, 2.5]
        return [_clamp(float(value), 0.1, 20.0) for value in configured]

    def target_age(self, arm):
        timestamp = self.last_valid_time[arm]
        return math.inf if timestamp is None else time.monotonic() - timestamp

    def _reload_config(self):
        try:
            mtime = self.config_path.stat().st_mtime
        except OSError:
            return
        if mtime <= self.config_mtime:
            return
        with self.config_path.open("r", encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream)
        self.config_mtime = mtime
        print(f"[OpenArmTaskStudio] Reloaded D435 mapping: {self.config_path}")

    def _map(self, arm, observation):
        arm_config = self.config["arms"][arm]
        config = arm_config["joint_pose"]
        positions = list(config["neutral_positions"])
        target = observation.get("target_position")
        if not target:
            return positions
        neutral = arm_config["neutral_position"]
        workspace_min = arm_config["workspace_min"]
        workspace_max = arm_config["workspace_max"]

        def interval(value, low, high):
            return 0.0 if abs(high - low) < 1e-6 else _clamp((value - low) / (high - low), 0.0, 1.0)

        forward = interval(float(target[0]), float(neutral[0]), float(workspace_max[0]))
        wrist_backward = interval(float(target[0]), float(neutral[0]), float(workspace_min[0]))
        upper_arm_backward = self._upper_arm_backward_amount(arm, observation, config)
        backward = max(wrist_backward, upper_arm_backward)
        upward = interval(float(target[2]), float(neutral[2]), float(workspace_max[2]))
        target_y, neutral_y = float(target[1]), float(neutral[1])
        outward_y = float(workspace_max[1] if neutral_y >= 0.0 else workspace_min[1])
        inward_y = float(workspace_min[1] if neutral_y >= 0.0 else workspace_max[1])
        outward = interval(target_y, neutral_y, outward_y)
        inward = interval(target_y, neutral_y, inward_y)
        upper_outward = self._segment_amount(
            arm, observation.get("shoulder_camera"), observation.get("elbow_camera"),
            config, "upper_outward_start", "upper_outward_full", False,
        )
        forearm_inward = self._segment_amount(
            arm, observation.get("elbow_camera"), observation.get("wrist_camera"),
            config, "forearm_inward_start", "forearm_inward_full", True,
        )
        elbow_bend = self._elbow_bend_amount(observation, config)
        elbow_plane = self._elbow_plane_amount(arm, observation)
        positions[0] = _clamp(
            float(config.get("joint1_base", positions[0]))
            + float(config.get("joint1_forward_gain", 0.0)) * forward
            + float(config.get("joint1_backward_gain", 0.0)) * backward
            + float(config.get("joint1_up_gain", 0.0)) * upward,
            float(config.get("joint1_min", -2.0)), float(config.get("joint1_max", 2.0)),
        )
        positions[1] = _clamp(
            float(config.get("joint2_base", positions[1]))
            + float(config.get("joint2_outward_gain", 0.0)) * outward
            + float(config.get("joint2_inward_gain", 0.0)) * inward
            + float(config.get("joint2_upper_outward_gain", 0.0)) * upper_outward,
            float(config.get("joint2_min", -2.0)), float(config.get("joint2_max", 2.0)),
        )
        positions[2] = _clamp(
            float(config.get("joint3_base", positions[2]))
            + float(config.get("joint3_elbow_plane_gain", 0.0)) * elbow_plane,
            float(config.get("joint3_min", -2.0)), float(config.get("joint3_max", 2.0)),
        )
        positions[3] = _clamp(
            float(config.get("joint4_base", positions[3]))
            + float(config.get("joint4_forward_gain", 0.8)) * forward
            + float(config.get("joint4_up_gain", 0.0)) * upward
            + float(config.get("joint4_elbow_bend_gain", 0.0)) * elbow_bend,
            float(config.get("joint4_min", 0.0)), float(config.get("joint4_max", 2.0)),
        )
        positions[5] = _clamp(
            float(config.get("joint6_base", positions[5]))
            + float(config.get("joint6_forearm_inward_gain", 0.0)) * forearm_inward,
            float(config.get("joint6_min", -2.0)), float(config.get("joint6_max", 2.0)),
        )
        return [float(value) for value in positions]

    def _upper_arm_backward_amount(self, arm, observation, config):
        shoulder = observation.get("shoulder_camera")
        elbow = observation.get("elbow_camera")
        if shoulder is None or elbow is None:
            return 0.0
        vector = [float(elbow[index]) - float(shoulder[index]) for index in range(3)]
        robot_vector = self._camera_vector_to_robot(vector)
        norm = math.sqrt(sum(value * value for value in robot_vector))
        if norm < 1e-6:
            return 0.0
        backward_component = -robot_vector[0] / norm
        references = getattr(self, "upper_arm_backward_reference", None)
        if references is None:
            references = {side: None for side in ("left", "right")}
            self.upper_arm_backward_reference = references
        if references[arm] is None:
            references[arm] = backward_component
            return 0.0
        delta = backward_component - references[arm]
        start = float(config.get("upper_backward_start", 0.03))
        full = float(config.get("upper_backward_full", 0.55))
        if full <= start:
            return 0.0
        return _clamp((delta - start) / (full - start), 0.0, 1.0)

    def _elbow_plane_amount(self, arm, observation):
        shoulder = observation.get("shoulder_camera")
        elbow = observation.get("elbow_camera")
        wrist = observation.get("wrist_camera")
        if shoulder is None or elbow is None or wrist is None:
            return 0.0
        upper_camera = [float(elbow[index]) - float(shoulder[index]) for index in range(3)]
        forearm_camera = [float(wrist[index]) - float(elbow[index]) for index in range(3)]
        upper = self._camera_vector_to_robot(upper_camera)
        forearm = self._camera_vector_to_robot(forearm_camera)
        upper_norm = math.sqrt(sum(value * value for value in upper))
        forearm_norm = math.sqrt(sum(value * value for value in forearm))
        if upper_norm < 1e-6 or forearm_norm < 1e-6:
            return 0.0
        dot = sum(a * b for a, b in zip(upper, forearm)) / (upper_norm * forearm_norm)
        bend_visibility = math.sqrt(max(0.0, 1.0 - _clamp(dot, -1.0, 1.0) ** 2))
        side_sign = 1.0 if arm == "left" else -1.0
        signed_lateral = side_sign * forearm[1] / forearm_norm
        return _clamp(signed_lateral * bend_visibility, -1.0, 1.0)

    def _camera_vector_to_robot(self, vector):
        axis_map = self.config["mapping"]["camera_to_robot_delta"]
        values = {"camera_x": vector[0], "camera_y": vector[1], "camera_z": vector[2]}
        result = []
        for axis in ("x", "y", "z"):
            expression = axis_map[axis]
            sign = -1.0 if expression.startswith("-") else 1.0
            result.append(sign * values[expression.lstrip("-")])
        return result

    def _segment_amount(self, arm, start, end, config, start_key, full_key, inward):
        if start is None or end is None:
            return 0.0
        vector = [float(end[index]) - float(start[index]) for index in range(3)]
        norm = math.sqrt(sum(value * value for value in vector))
        if norm < 1e-6:
            return 0.0
        axis_map = self.config["mapping"]["camera_to_robot_delta"]
        values = {"camera_x": vector[0], "camera_y": vector[1], "camera_z": vector[2]}
        expression = axis_map["y"]
        lateral = (-1.0 if expression.startswith("-") else 1.0) * values[expression.lstrip("-")]
        signed = (1.0 if arm == "left" else -1.0) * lateral / norm
        if inward:
            signed = -signed
        start_value = float(config.get(start_key, 0.15))
        full_value = float(config.get(full_key, 0.75))
        return 0.0 if full_value <= start_value else _clamp((signed - start_value) / (full_value - start_value), 0.0, 1.0)

    @staticmethod
    def _elbow_bend_amount(observation, config):
        shoulder, elbow, wrist = (observation.get(key) for key in ("shoulder_camera", "elbow_camera", "wrist_camera"))
        if shoulder is None or elbow is None or wrist is None:
            return 0.0
        upper = [float(shoulder[index]) - float(elbow[index]) for index in range(3)]
        forearm = [float(wrist[index]) - float(elbow[index]) for index in range(3)]
        upper_norm = math.sqrt(sum(value * value for value in upper))
        forearm_norm = math.sqrt(sum(value * value for value in forearm))
        if upper_norm < 1e-6 or forearm_norm < 1e-6:
            return 0.0
        dot = sum(a * b for a, b in zip(upper, forearm)) / (upper_norm * forearm_norm)
        elbow_angle_deg = math.degrees(math.acos(_clamp(dot, -1.0, 1.0)))
        straight_deg = float(config.get("elbow_straight_deg", 165.0))
        bent_deg = float(config.get("elbow_bent_deg", 70.0))
        if straight_deg <= bent_deg:
            return 0.0
        return _clamp((straight_deg - elbow_angle_deg) / (straight_deg - bent_deg), 0.0, 1.0)

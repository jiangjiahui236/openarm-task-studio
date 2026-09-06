#!/usr/bin/env python3
"""Task Studio RGB-D/RGB upper-body tracking sender."""

from __future__ import annotations

import argparse
import json
import math
import socket
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

import yaml

import cv2
import numpy as np
try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


SIDES = ("left", "right")


@dataclass
class ArmObservation:
    shoulder: np.ndarray
    elbow: np.ndarray
    wrist: np.ndarray
    confidence: float
    forearm_dir: np.ndarray
    upper_arm_dir: np.ndarray
    tracking_status: str = "live"


class ArmObservationStabilizer:
    """Reject implausible RGB-D geometry and bridge very short depth dropouts."""

    def __init__(self, config: dict):
        self.last = {side: None for side in SIDES}
        self.last_time = {side: None for side in SIDES}
        self.segment_reference = {side: None for side in SIDES}
        self.update_config(config)

    def update_config(self, config: dict) -> None:
        tracking = config.get("tracking", {})
        self.min_segment_m = float(tracking.get("min_arm_segment_m", 0.12))
        self.reference_min_segment_m = float(
            tracking.get("reference_min_arm_segment_m", 0.12)
        )
        self.max_segment_m = float(tracking.get("max_arm_segment_m", 0.65))
        self.segment_ratio_min = float(tracking.get("segment_ratio_min", 0.55))
        self.segment_ratio_max = float(tracking.get("segment_ratio_max", 1.65))
        self.max_point_speed_m_s = float(tracking.get("max_point_speed_m_s", 3.0))
        self.jump_allowance_m = float(tracking.get("point_jump_allowance_m", 0.04))
        self.position_alpha = float(tracking.get("position_smoothing_alpha", 0.55))
        self.dropout_hold_s = float(tracking.get("depth_dropout_hold_s", 0.15))

    @staticmethod
    def _segments(arm: ArmObservation) -> np.ndarray:
        return np.array([
            np.linalg.norm(arm.elbow - arm.shoulder),
            np.linalg.norm(arm.wrist - arm.elbow),
        ], dtype=float)

    @staticmethod
    def _with_points(arm: ArmObservation, points, status: str, confidence=None) -> ArmObservation:
        shoulder, elbow, wrist = (np.asarray(point, dtype=float) for point in points)
        return ArmObservation(
            shoulder=shoulder,
            elbow=elbow,
            wrist=wrist,
            confidence=float(arm.confidence if confidence is None else confidence),
            forearm_dir=normalize(wrist - elbow),
            upper_arm_dir=normalize(elbow - shoulder),
            tracking_status=status,
        )

    def _hold(self, side: str, now: float, reason: str) -> Optional[ArmObservation]:
        previous = self.last[side]
        previous_time = self.last_time[side]
        if previous is None or previous_time is None or now - previous_time > self.dropout_hold_s:
            return None
        confidence = max(0.45, float(previous.confidence) * 0.85)
        return self._with_points(
            previous,
            (previous.shoulder, previous.elbow, previous.wrist),
            f"held:{reason}",
            confidence,
        )

    def update(self, observations: Dict[str, ArmObservation], now: float) -> Dict[str, ArmObservation]:
        output: Dict[str, ArmObservation] = {}
        for side in SIDES:
            candidate = observations.get(side)
            if candidate is None:
                held = self._hold(side, now, "depth")
                if held is not None:
                    output[side] = held
                continue

            segments = self._segments(candidate)
            valid = bool(np.all(np.isfinite(segments))) and bool(
                np.all((segments >= self.min_segment_m) & (segments <= self.max_segment_m))
            )
            reference = self.segment_reference[side]
            if valid and reference is not None:
                ratios = segments / np.maximum(reference, 1.0e-6)
                valid = bool(np.all((ratios >= self.segment_ratio_min) & (ratios <= self.segment_ratio_max)))

            previous = self.last[side]
            previous_time = self.last_time[side]
            if valid and previous is not None and previous_time is not None:
                dt = max(1.0 / 120.0, now - previous_time)
                allowed_jump = self.jump_allowance_m + self.max_point_speed_m_s * dt
                jumps = [
                    np.linalg.norm(candidate.shoulder - previous.shoulder),
                    np.linalg.norm(candidate.elbow - previous.elbow),
                    np.linalg.norm(candidate.wrist - previous.wrist),
                ]
                valid = max(jumps) <= allowed_jump

            if not valid:
                held = self._hold(side, now, "geometry")
                if held is not None:
                    output[side] = held
                continue

            if previous is not None:
                alpha = min(1.0, max(0.05, self.position_alpha))
                points = [
                    alpha * current + (1.0 - alpha) * old
                    for current, old in zip(
                        (candidate.shoulder, candidate.elbow, candidate.wrist),
                        (previous.shoulder, previous.elbow, previous.wrist),
                    )
                ]
                candidate = self._with_points(candidate, points, "live")
            self.last[side] = candidate
            self.last_time[side] = now
            # 与 openarm_studio 对齐：前伸导致的透视缩短短肢段可用于实时控制，
            # 但只有两段均不小于 reference_min_segment_m 且处于已有参考
            # 70%-130% 时才允许建立/缓慢更新臂长参考，避免污染参考后假 miss。
            reference_eligible = bool(np.all(segments >= self.reference_min_segment_m))
            if reference is None and reference_eligible:
                self.segment_reference[side] = segments
            elif reference is not None and reference_eligible:
                stable_ratios = segments / np.maximum(reference, 1.0e-6)
                if bool(np.all((stable_ratios >= 0.70) & (stable_ratios <= 1.30))):
                    self.segment_reference[side] = 0.98 * reference + 0.02 * segments
            output[side] = candidate
        return output


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Preview RGB-D/RGB + MediaPipe upper-body arm tracking")
    parser.add_argument("--source", choices=("rgbd", "rgb"), default="rgbd")
    parser.add_argument("--camera-index", type=int, default=0, help="OpenCV camera index used by RGB mode")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--no-preview", action="store_true")
    parser.add_argument("--preview-file", type=Path, help="Write the latest annotated JPEG for an embedded UI")
    parser.add_argument("--preview-every", type=int, default=2, help="Write one embedded preview frame every N frames")
    parser.add_argument("--frames", type=int, default=0, help="Stop after N frames; 0 means run until q/Esc")
    parser.add_argument("--min-visibility", type=float, default=0.55)
    parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parent / "config" / "openarm_d435_teleop_visual.yaml")
    parser.add_argument("--calibration-frames", type=int, default=45)
    parser.add_argument("--udp-host", default="", help="Send mapped targets to this host; empty disables UDP")
    parser.add_argument("--udp-port", type=int, default=5005)
    return parser.parse_args()


def normalize(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1.0e-6:
        return np.zeros(3)
    return v / n




def orthonormalize(x_axis: np.ndarray, y_hint: np.ndarray) -> np.ndarray:
    x_axis = normalize(x_axis)
    if float(np.linalg.norm(x_axis)) < 1.0e-6:
        x_axis = np.array([1.0, 0.0, 0.0])
    y_axis = y_hint - np.dot(y_hint, x_axis) * x_axis
    y_axis = normalize(y_axis)
    if float(np.linalg.norm(y_axis)) < 1.0e-6:
        y_axis = np.array([0.0, 1.0, 0.0])
    z_axis = normalize(np.cross(x_axis, y_axis))
    if float(np.linalg.norm(z_axis)) < 1.0e-6:
        z_axis = np.array([0.0, 0.0, 1.0])
    y_axis = normalize(np.cross(z_axis, x_axis))
    return np.column_stack((x_axis, y_axis, z_axis))


def rotmat_to_quat(rot: np.ndarray) -> np.ndarray:
    m = rot
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=float)
    return q / max(float(np.linalg.norm(q)), 1.0e-9)


def quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    q = q / max(float(np.linalg.norm(q)), 1.0e-9)
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=float)


def arm_rotation(arm: ArmObservation) -> np.ndarray:
    y_hint = np.cross(arm.forearm_dir, arm.upper_arm_dir)
    return orthonormalize(arm.forearm_dir, y_hint)


def load_config(path: Path) -> dict:
    with path.open('r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def maybe_reload_config(path: Path, config: dict, last_mtime: Optional[float]) -> Tuple[dict, Optional[float]]:
    try:
        mtime = path.stat().st_mtime
    except OSError:
        return config, last_mtime
    if last_mtime is not None and mtime <= last_mtime:
        return config, last_mtime
    try:
        new_config = load_config(path)
    except Exception as exc:
        print(f"Config reload ignored: {exc}", flush=True)
        return config, last_mtime
    print(f"Reloaded config: {path}", flush=True)
    return new_config, mtime


def axis_expr(expr: str, v: np.ndarray) -> float:
    sign = -1.0 if expr.startswith('-') else 1.0
    name = expr[1:] if expr.startswith('-') else expr
    return sign * float(v[{"camera_x": 0, "camera_y": 1, "camera_z": 2}[name]])


def camera_rotation_to_robot(rot_camera: np.ndarray, axis_map: dict) -> np.ndarray:
    basis = np.zeros((3, 3), dtype=float)
    for i, axis in enumerate(("x", "y", "z")):
        expr = axis_map[axis]
        sign = -1.0 if expr.startswith('-') else 1.0
        name = expr[1:] if expr.startswith('-') else expr
        basis[i, {"camera_x": 0, "camera_y": 1, "camera_z": 2}[name]] = sign
    return basis @ rot_camera @ basis.T


def build_udp_payload(
    config: dict,
    frame: int,
    fps: float,
    arms: Dict[str, ArmObservation],
    neutral_wrist: Dict[str, np.ndarray],
    neutral_rot: Dict[str, np.ndarray],
) -> dict:
    axis_map = config['mapping']['camera_to_robot_delta']
    scale = np.array(config['mapping']['position_scale'], dtype=float)
    max_delta = np.array(config['mapping']['max_delta_m'], dtype=float)
    payload = {
        'type': 'openarm_arm_pose_targets',
        'timestamp': time.time(),
        'frame': frame,
        'fps': fps,
        'base_frame': config['robot']['base_frame'],
        'calibrated': all(side in neutral_wrist for side in SIDES),
        'arms': {},
    }
    for side, arm in arms.items():
        arm_payload = {
            'confidence': float(arm.confidence),
            'tracking_status': arm.tracking_status,
            'wrist_camera': arm.wrist.tolist(),
            'elbow_camera': arm.elbow.tolist(),
            'shoulder_camera': arm.shoulder.tolist(),
            'forearm_dir_camera': arm.forearm_dir.tolist(),
            'upper_arm_dir_camera': arm.upper_arm_dir.tolist(),
        }
        if payload['calibrated'] and side in neutral_wrist:
            raw_delta = arm.wrist - neutral_wrist[side]
            robot_delta = np.array([
                axis_expr(axis_map['x'], raw_delta),
                axis_expr(axis_map['y'], raw_delta),
                axis_expr(axis_map['z'], raw_delta),
            ], dtype=float)
            robot_delta = np.clip(robot_delta * scale, -max_delta, max_delta)
            arm_cfg = config['arms'][side]
            neutral_position = np.array(arm_cfg['neutral_position'], dtype=float)
            workspace_min = np.array(arm_cfg['workspace_min'], dtype=float)
            workspace_max = np.array(arm_cfg['workspace_max'], dtype=float)
            position = np.clip(neutral_position + robot_delta, workspace_min, workspace_max)
            neutral_quat = np.array(arm_cfg['neutral_quaternion_xyzw'], dtype=float)
            orientation_mode = config.get('mapping', {}).get('orientation_mode', 'human_delta')
            if orientation_mode == 'fixed':
                quat = neutral_quat
            else:
                human_delta = arm_rotation(arm) @ neutral_rot[side].T
                robot_delta_rot = camera_rotation_to_robot(human_delta, axis_map)
                neutral_robot_rot = quat_to_rotmat(neutral_quat)
                quat = rotmat_to_quat(robot_delta_rot @ neutral_robot_rot)
            arm_payload['target_position'] = position.tolist()
            arm_payload['target_quaternion_xyzw'] = quat.tolist()
        payload['arms'][side] = arm_payload
    return payload

def start_realsense(width: int, height: int, fps: int) -> Tuple[rs.pipeline, rs.align]:
    if rs is None:
        raise RuntimeError("pyrealsense2 is required for RGB-D mode; select RGB for a regular webcam")
    last_error = None
    for attempt in range(2):
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        try:
            pipe.start(cfg)
            align = rs.align(rs.stream.color)
            for _ in range(3):
                pipe.wait_for_frames(5000)
            return pipe, align
        except RuntimeError as exc:
            last_error = exc
            try:
                pipe.stop()
            except RuntimeError:
                pass
            if attempt == 0:
                print("D435 did not deliver frames; requesting hardware reset and retrying...", flush=True)
                for dev in rs.context().query_devices():
                    dev.hardware_reset()
                time.sleep(5.0)
    raise RuntimeError(f"Could not start D435 color+depth streams: {last_error}")


def create_depth_filters(config: dict):
    tracking = config.get("tracking", {})
    if not bool(tracking.get("enable_depth_filters", True)):
        return []
    spatial = rs.spatial_filter()
    spatial.set_option(rs.option.filter_magnitude, 2)
    spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
    spatial.set_option(rs.option.filter_smooth_delta, 20)
    temporal = rs.temporal_filter()
    temporal.set_option(
        rs.option.filter_smooth_alpha,
        float(tracking.get("depth_temporal_alpha", 0.8)),
    )
    temporal.set_option(
        rs.option.filter_smooth_delta,
        float(tracking.get("depth_temporal_delta", 30)),
    )
    holes = rs.hole_filling_filter(1)
    return [spatial, temporal, holes]


def apply_depth_filters(depth_frame, filters):
    filtered = depth_frame
    for depth_filter in filters:
        filtered = depth_filter.process(filtered)
    return filtered.as_depth_frame()


def deproject(depth_frame, intrinsics, px: int, py: int, window: int = 3) -> Optional[np.ndarray]:
    distances = []
    for dy in range(-window, window + 1):
        for dx in range(-window, window + 1):
            x = px + dx
            y = py + dy
            if 0 <= x < intrinsics.width and 0 <= y < intrinsics.height:
                d = float(depth_frame.get_distance(x, y))
                if 0.15 < d < 4.0:
                    distances.append(d)
    if not distances:
        return None
    depth = float(np.median(distances))
    return np.array(rs.rs2_deproject_pixel_to_point(intrinsics, [px, py], depth), dtype=float)


def landmark_point(lm, landmark_id, image_shape, depth_frame, intrinsics, min_visibility: float):
    mark = lm[landmark_id.value]
    visibility = float(getattr(mark, "visibility", 1.0))
    if visibility < min_visibility:
        return None, visibility
    h, w = image_shape[:2]
    px = int(np.clip(round(mark.x * w), 0, w - 1))
    py = int(np.clip(round(mark.y * h), 0, h - 1))
    point = deproject(depth_frame, intrinsics, px, py)
    if point is None:
        return None, 0.0
    return point, visibility


def extract_arms(result, image_shape, depth_frame, intrinsics, min_visibility: float) -> Dict[str, ArmObservation]:
    if result.pose_landmarks is None:
        return {}
    import mediapipe as mp

    pose = mp.solutions.pose.PoseLandmark
    lm = result.pose_landmarks.landmark
    arms: Dict[str, ArmObservation] = {}
    for side in SIDES:
        prefix = "LEFT" if side == "left" else "RIGHT"
        shoulder, vs = landmark_point(lm, getattr(pose, f"{prefix}_SHOULDER"), image_shape, depth_frame, intrinsics, min_visibility)
        elbow, ve = landmark_point(lm, getattr(pose, f"{prefix}_ELBOW"), image_shape, depth_frame, intrinsics, min_visibility)
        wrist, vw = landmark_point(lm, getattr(pose, f"{prefix}_WRIST"), image_shape, depth_frame, intrinsics, min_visibility)
        if shoulder is None or elbow is None or wrist is None:
            continue
        arms[side] = ArmObservation(
            shoulder=shoulder,
            elbow=elbow,
            wrist=wrist,
            confidence=min(vs, ve, vw),
            forearm_dir=normalize(wrist - elbow),
            upper_arm_dir=normalize(elbow - shoulder),
        )
    return arms


def extract_rgb_arms(result, min_visibility: float) -> Dict[str, ArmObservation]:
    """Build camera-oriented 3-D arm observations from MediaPipe monocular world landmarks."""
    if result.pose_landmarks is None or result.pose_world_landmarks is None:
        return {}
    import mediapipe as mp

    pose = mp.solutions.pose.PoseLandmark
    image_landmarks = result.pose_landmarks.landmark
    world_landmarks = result.pose_world_landmarks.landmark
    arms: Dict[str, ArmObservation] = {}
    for side in SIDES:
        prefix = "LEFT" if side == "left" else "RIGHT"
        landmark_ids = [
            getattr(pose, f"{prefix}_SHOULDER"),
            getattr(pose, f"{prefix}_ELBOW"),
            getattr(pose, f"{prefix}_WRIST"),
        ]
        visibility = min(float(getattr(image_landmarks[item.value], "visibility", 1.0)) for item in landmark_ids)
        if visibility < min_visibility:
            continue
        shoulder, elbow, wrist = (
            np.array([world_landmarks[item.value].x, world_landmarks[item.value].y, world_landmarks[item.value].z], dtype=float)
            for item in landmark_ids
        )
        arms[side] = ArmObservation(
            shoulder=shoulder,
            elbow=elbow,
            wrist=wrist,
            confidence=visibility,
            forearm_dir=normalize(wrist - elbow),
            upper_arm_dir=normalize(elbow - shoulder),
        )
    return arms


def draw(color, result, arms: Dict[str, ArmObservation], fps: float, payload: Optional[dict], source: str) -> None:
    import mediapipe as mp

    if result.pose_landmarks:
        mp.solutions.drawing_utils.draw_landmarks(
            color,
            result.pose_landmarks,
            mp.solutions.pose.POSE_CONNECTIONS,
        )
    y = 28
    cv2.putText(color, f"{source.upper()} arm pose {fps:.1f} FPS", (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (40, 240, 40), 2)
    y += 28
    for side in SIDES:
        if side not in arms:
            text = f"{side}: not visible"
            color_text = (40, 40, 255)
        else:
            arm = arms[side]
            w = arm.wrist
            text = f"{side}: {arm.tracking_status} cam(right={w[0]:+.2f},down={w[1]:+.2f},depth={w[2]:+.2f}) conf={arm.confidence:.2f}"
            if payload is not None:
                arm_payload = payload.get("arms", {}).get(side, {})
                target = arm_payload.get("target_position")
                if target is not None:
                    text += f" robot(x={target[0]:+.2f},y={target[1]:+.2f},z={target[2]:+.2f})"
            color_text = (255, 255, 255)
        cv2.putText(color, text, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color_text, 1)
        y += 24


def main() -> None:
    args = parse_args()
    config = load_config(args.config)
    try:
        config_mtime = args.config.stat().st_mtime
    except OSError:
        config_mtime = None
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if args.udp_host else None
    neutral_samples = []
    neutral_wrist: Dict[str, np.ndarray] = {}
    neutral_rot: Dict[str, np.ndarray] = {}
    stabilizer = ArmObservationStabilizer(config)

    pipe = align = capture = None
    depth_filters = []
    if args.source == "rgbd":
        print("Starting D435 RGB-D capture before importing MediaPipe...", flush=True)
        pipe, align = start_realsense(args.width, args.height, args.fps)
        depth_filters = create_depth_filters(config)
    else:
        print(f"Starting RGB camera index {args.camera_index}...", flush=True)
        capture = cv2.VideoCapture(args.camera_index)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        capture.set(cv2.CAP_PROP_FPS, args.fps)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f"Could not open RGB camera index {args.camera_index}")

    import mediapipe as mp

    pose_tracker = mp.solutions.pose.Pose(
        model_complexity=1,
        smooth_landmarks=True,
        min_detection_confidence=0.55,
        min_tracking_confidence=0.55,
    )

    count = 0
    last_t = time.monotonic()
    fps = 0.0
    try:
        while True:
            config, config_mtime = maybe_reload_config(args.config, config, config_mtime)
            stabilizer.update_config(config)
            if args.source == "rgbd":
                frames = align.process(pipe.wait_for_frames(5000))
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue
                depth_frame = apply_depth_filters(depth_frame, depth_filters)
                color = np.asanyarray(color_frame.get_data())
                intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
            else:
                frame_ok, color = capture.read()
                if not frame_ok:
                    continue
            rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
            rgb.flags.writeable = False
            result = pose_tracker.process(rgb)
            raw_arms = (
                extract_arms(result, color.shape, depth_frame, intrinsics, args.min_visibility)
                if args.source == "rgbd"
                else extract_rgb_arms(result, args.min_visibility)
            )
            arms = stabilizer.update(raw_arms, time.monotonic())

            if len(neutral_wrist) < len(SIDES):
                if all(side in arms for side in SIDES):
                    neutral_samples.append({side: arms[side] for side in SIDES})
                    if len(neutral_samples) >= args.calibration_frames:
                        for side in SIDES:
                            neutral_wrist[side] = np.mean([sample[side].wrist for sample in neutral_samples], axis=0)
                            neutral_rot[side] = arm_rotation(neutral_samples[-1][side])
                        print("Neutral pose calibrated for UDP targets.", flush=True)
                else:
                    neutral_samples.clear()

            now = time.monotonic()
            dt = now - last_t
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps else 1.0 / dt
            last_t = now

            count += 1
            payload = None
            if udp_sock is not None:
                payload = build_udp_payload(config, count, fps, arms, neutral_wrist, neutral_rot)
                udp_sock.sendto(json.dumps(payload, separators=(",", ":")).encode("utf-8"), (args.udp_host, args.udp_port))

            if count % 30 == 0 or args.frames:
                parts = []
                for side in SIDES:
                    if side not in arms:
                        parts.append(f"{side}:missing")
                        continue
                    arm = arms[side]
                    part = (
                        f"{side}:{arm.tracking_status} conf={arm.confidence:.2f}"
                        f" cam=(right={arm.wrist[0]:+.2f},down={arm.wrist[1]:+.2f},depth={arm.wrist[2]:+.2f})"
                    )
                    if payload is not None:
                        arm_payload = payload.get("arms", {}).get(side, {})
                        target = arm_payload.get("target_position")
                        if target is not None:
                            part += f" robot=(x={target[0]:+.2f},y={target[1]:+.2f},z={target[2]:+.2f})"
                    parts.append(part)
                summary = " ".join(parts)
                udp_text = " udp" if udp_sock is not None else ""
                cal_text = " calibrated" if len(neutral_wrist) == len(SIDES) else f" calibrating={len(neutral_samples)}/{args.calibration_frames}"
                print(f"frame {count}: {summary}{udp_text}{cal_text}", flush=True)

            if not args.no_preview or args.preview_file:
                draw(color, result, arms, fps, payload, args.source)
            if args.preview_file and count % max(1, args.preview_every) == 0:
                args.preview_file.parent.mkdir(parents=True, exist_ok=True)
                temporary = args.preview_file.with_suffix(".tmp.jpg")
                if cv2.imwrite(str(temporary), color, [cv2.IMWRITE_JPEG_QUALITY, 82]):
                    temporary.replace(args.preview_file)
            if not args.no_preview:
                cv2.imshow("openarm_motion_capture_preview", color)
                key = cv2.waitKey(1)
                if key in (27, ord("q")):
                    break
            if args.frames and count >= args.frames:
                break
    finally:
        pose_tracker.close()
        if pipe is not None:
            pipe.stop()
        if capture is not None:
            capture.release()
        if udp_sock is not None:
            udp_sock.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

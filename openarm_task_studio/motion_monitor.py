from __future__ import annotations

import json
import math
import time
from pathlib import Path


DEFAULT_LOG_DIR = Path(__file__).resolve().parents[1] / "logs" / "motion_capture"


class MotionExecutionMonitor:
    def __init__(self, log_dir=DEFAULT_LOG_DIR):
        self.log_dir = Path(log_dir)
        self.stream = None
        self.path = None
        self.started_at = None
        self.mode = None
        self.sample_count = 0
        self.filtered_spike_start = 0
        self.metrics = {arm: self._empty_metrics() for arm in ("left", "right")}

    @staticmethod
    def _empty_metrics():
        return {
            "samples": 0,
            "packet_age_ms_sum": 0.0,
            "packet_age_ms_max": 0.0,
            "mapped_error_rad_sum": 0.0,
            "mapped_error_rad_max": 0.0,
            "command_error_rad_sum": 0.0,
            "command_error_rad_max": 0.0,
        }

    @property
    def active(self):
        return self.stream is not None

    def start(self, mode, filtered_spike_count=0):
        self.stop(filtered_spike_count)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.path = self.log_dir / f"motion_{stamp}_{mode}.jsonl"
        self.stream = self.path.open("w", encoding="utf-8", buffering=1)
        self.started_at = time.time()
        self.mode = mode
        self.sample_count = 0
        self.filtered_spike_start = int(filtered_spike_count)
        self.metrics = {arm: self._empty_metrics() for arm in ("left", "right")}
        self.stream.write(json.dumps({
            "type": "session_start",
            "schema": "openarm.motion-monitor/v1",
            "timestamp": self.started_at,
            "mode": mode,
        }, separators=(",", ":")) + "\n")
        return self.path

    def record(self, arm, mapped_target, commanded_target, actual, metadata, filtered_spike_count):
        if not self.active:
            return None
        metadata = metadata or {}
        now = time.time()
        source_timestamp = metadata.get("source_timestamp")
        packet_age_ms = (
            max(0.0, (now - float(source_timestamp)) * 1000.0)
            if source_timestamp is not None else math.nan
        )
        mapped_error = max(abs(float(a) - float(b)) for a, b in zip(mapped_target, actual))
        command_error = max(abs(float(a) - float(b)) for a, b in zip(commanded_target, actual))
        sample = {
            "type": "sample",
            "timestamp": now,
            "elapsed_s": now - self.started_at,
            "arm": arm,
            "source_frame": metadata.get("source_frame"),
            "source_fps": metadata.get("source_fps"),
            "tracking_status": metadata.get("tracking_status", "unknown"),
            "tracking_confidence": metadata.get("confidence"),
            "shoulder_camera": metadata.get("shoulder_camera"),
            "elbow_camera": metadata.get("elbow_camera"),
            "wrist_camera": metadata.get("wrist_camera"),
            "mapped_tcp_target": metadata.get("target_position"),
            "packet_age_ms": None if math.isnan(packet_age_ms) else packet_age_ms,
            "packet_accepted": metadata.get("accepted"),
            "mapped_target": [float(value) for value in mapped_target],
            "commanded_target": [float(value) for value in commanded_target],
            "actual_joint_positions": [float(value) for value in actual],
            "mapped_max_error_rad": mapped_error,
            "command_max_error_rad": command_error,
            "filtered_spike_count": int(filtered_spike_count),
        }
        self.stream.write(json.dumps(sample, separators=(",", ":")) + "\n")
        self.sample_count += 1
        metrics = self.metrics[arm]
        metrics["samples"] += 1
        if not math.isnan(packet_age_ms):
            metrics["packet_age_ms_sum"] += packet_age_ms
            metrics["packet_age_ms_max"] = max(metrics["packet_age_ms_max"], packet_age_ms)
        metrics["mapped_error_rad_sum"] += mapped_error
        metrics["mapped_error_rad_max"] = max(metrics["mapped_error_rad_max"], mapped_error)
        metrics["command_error_rad_sum"] += command_error
        metrics["command_error_rad_max"] = max(metrics["command_error_rad_max"], command_error)
        return sample

    def snapshot(self):
        arms = {}
        for arm, values in self.metrics.items():
            count = values["samples"]
            arms[arm] = {
                "samples": count,
                "packet_age_ms_mean": values["packet_age_ms_sum"] / count if count else 0.0,
                "packet_age_ms_max": values["packet_age_ms_max"],
                "mapped_error_rad_mean": values["mapped_error_rad_sum"] / count if count else 0.0,
                "mapped_error_rad_max": values["mapped_error_rad_max"],
                "command_error_rad_mean": values["command_error_rad_sum"] / count if count else 0.0,
                "command_error_rad_max": values["command_error_rad_max"],
            }
        return {"path": str(self.path) if self.path else "", "samples": self.sample_count, "arms": arms}

    def stop(self, filtered_spike_count=0):
        if not self.active:
            return None
        stopped_at = time.time()
        summary = self.snapshot()
        summary.update({
            "schema": "openarm.motion-monitor-summary/v1",
            "mode": self.mode,
            "started_at": self.started_at,
            "stopped_at": stopped_at,
            "duration_s": stopped_at - self.started_at,
            "filtered_spikes": max(0, int(filtered_spike_count) - self.filtered_spike_start),
        })
        self.stream.write(json.dumps({"type": "session_stop", **summary}, separators=(",", ":")) + "\n")
        self.stream.close()
        self.stream = None
        summary_path = self.path.with_suffix(".summary.json")
        summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        return summary_path

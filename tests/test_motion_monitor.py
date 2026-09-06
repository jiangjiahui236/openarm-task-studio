import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "openarm_task_studio" / "motion_monitor.py"
SPEC = importlib.util.spec_from_file_location("motion_monitor_test", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MotionExecutionMonitorTest(unittest.TestCase):
    def test_session_records_samples_and_writes_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            monitor = MODULE.MotionExecutionMonitor(directory)
            path = monitor.start("left", filtered_spike_count=4)
            sample = monitor.record(
                "left",
                [1.0, 0.0],
                [0.8, 0.0],
                [0.5, 0.0],
                {
                    "source_timestamp": MODULE.time.time() - 0.02,
                    "source_frame": 42,
                    "source_fps": 30.0,
                    "tracking_status": "live",
                    "confidence": 0.92,
                    "shoulder_camera": [0.0, 0.0, 1.0],
                    "elbow_camera": [0.0, 0.2, 1.0],
                    "wrist_camera": [0.0, 0.4, 1.0],
                    "target_position": [0.1, 0.2, 0.3],
                    "accepted": True,
                },
                filtered_spike_count=5,
            )
            self.assertEqual(sample["source_frame"], 42)
            self.assertEqual(sample["wrist_camera"], [0.0, 0.4, 1.0])
            self.assertEqual(sample["mapped_tcp_target"], [0.1, 0.2, 0.3])
            self.assertAlmostEqual(sample["mapped_max_error_rad"], 0.5)
            self.assertAlmostEqual(sample["command_max_error_rad"], 0.3)
            summary_path = monitor.stop(filtered_spike_count=7)
            self.assertTrue(path.is_file())
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(summary["samples"], 1)
            self.assertEqual(summary["filtered_spikes"], 3)
            self.assertAlmostEqual(summary["arms"]["left"]["mapped_error_rad_max"], 0.5)
            records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
            self.assertEqual([record["type"] for record in records], ["session_start", "sample", "session_stop"])


if __name__ == "__main__":
    unittest.main()

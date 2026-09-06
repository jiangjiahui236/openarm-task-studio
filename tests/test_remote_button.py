import importlib.util
import socket
import time
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "openarm_task_studio" / "remote_button.py"
SPEC = importlib.util.spec_from_file_location("remote_button_test", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class RemoteRecordButtonTest(unittest.TestCase):
    def test_accepts_plain_and_json_record_messages(self):
        check = MODULE.RemoteRecordButtonReceiver._is_record_message
        self.assertTrue(check(b"RECORD\n"))
        self.assertTrue(check(b"RECORD:42"))
        self.assertTrue(check(b'{"action":"record"}'))
        self.assertFalse(check(b"release"))

    def test_sequence_packets_are_acknowledged_and_counted_once(self):
        receiver = MODULE.RemoteRecordButtonReceiver(host="127.0.0.1", port=0, advertise=False)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sender.settimeout(1.0)
        try:
            address = receiver.socket.getsockname()
            sender.sendto(b"RECORD:7", address)
            sender.sendto(b"RECORD:7", address)
            deadline = time.monotonic() + 1.0
            while receiver.poll() < 1 and time.monotonic() < deadline:
                time.sleep(0.01)
            acknowledgements = {sender.recvfrom(64)[0], sender.recvfrom(64)[0]}
            self.assertEqual(acknowledgements, {b"ACK:7"})
            self.assertEqual(receiver.press_count, 1)
        finally:
            sender.close()
            receiver.close()

    def test_heartbeat_reports_connection_and_button_state(self):
        receiver = MODULE.RemoteRecordButtonReceiver(host="127.0.0.1", port=0, advertise=False)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sender.settimeout(1.0)
        try:
            address = receiver.socket.getsockname()
            sender.sendto(b"PING:DOWN", address)
            receiver.poll()
            self.assertEqual(sender.recvfrom(64)[0], b"PONG")
            status = receiver.status()
            self.assertTrue(status["is_down"])
            self.assertEqual(status["address"][0], "127.0.0.1")
            sender.sendto(b"PING:UP", address)
            receiver.poll()
            self.assertFalse(receiver.status()["is_down"])
            self.assertEqual(receiver.press_count, 0)
        finally:
            sender.close()
            receiver.close()

    def test_receiver_can_disable_service_advertisement(self):
        receiver = MODULE.RemoteRecordButtonReceiver(host="127.0.0.1", port=0, advertise=False)
        try:
            self.assertIsNone(receiver._publisher)
            self.assertIsInstance(receiver.socket, socket.socket)
        finally:
            receiver.close()


if __name__ == "__main__":
    unittest.main()

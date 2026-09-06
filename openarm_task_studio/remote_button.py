from __future__ import annotations

import json
import shutil
import socket
import subprocess
import time


class RemoteRecordButtonReceiver:
    SERVICE_TYPE = "_openarm-record._udp"

    def __init__(self, host="0.0.0.0", port=5011, debounce_seconds=0.25, advertise=True):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((host, port))
        self.socket.setblocking(False)
        self.debounce_seconds = debounce_seconds
        self.last_press_time = 0.0
        self.press_count = 0
        self.last_packet_time = 0.0
        self.last_button_time = 0.0
        self.last_address = None
        self.button_is_down = False
        self.last_sequence = None
        self._seen_sequences = {}
        self._publisher = self._start_service_publisher(port) if advertise else None

    @classmethod
    def _start_service_publisher(cls, port):
        executable = shutil.which("avahi-publish-service")
        if executable is None:
            print("[OpenArmTaskStudio] Warning: avahi-publish-service is unavailable; remote button mDNS discovery is disabled")
            return None
        try:
            publisher = subprocess.Popen(
                [executable, "-s", "OpenArm Task Studio", cls.SERVICE_TYPE, str(port)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except OSError as exc:
            print(f"[OpenArmTaskStudio] Warning: failed to publish remote button mDNS service: {exc}")
            return None
        print(f"[OpenArmTaskStudio] Remote button mDNS service ready: {cls.SERVICE_TYPE} port {port}")
        return publisher

    def close(self):
        if self._publisher is not None and self._publisher.poll() is None:
            self._publisher.terminate()
            try:
                self._publisher.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self._publisher.kill()
                self._publisher.wait()
        self._publisher = None
        self.socket.close()

    def poll(self):
        while True:
            try:
                data, address = self.socket.recvfrom(1024)
            except BlockingIOError:
                break
            now = time.monotonic()
            try:
                text = data.decode("utf-8").strip().upper()
            except UnicodeDecodeError:
                continue
            if text in {"PING:UP", "PING:DOWN"}:
                self.last_packet_time = now
                self.last_address = address
                self.button_is_down = text.endswith(":DOWN")
                try:
                    self.socket.sendto(b"PONG", address)
                except OSError:
                    pass
                continue
            sequence = self._record_sequence(data)
            if sequence is False:
                continue
            self.last_packet_time = now
            self.last_button_time = now
            self.last_address = address
            self.button_is_down = True
            self.last_sequence = sequence
            if sequence is not None:
                try:
                    self.socket.sendto(f"ACK:{sequence}".encode("ascii"), address)
                except OSError:
                    pass
                sender_key = (address[0], sequence)
                if sender_key in self._seen_sequences:
                    continue
                self._seen_sequences[sender_key] = now
                if len(self._seen_sequences) > 256:
                    cutoff = now - 300.0
                    self._seen_sequences = {
                        key: timestamp for key, timestamp in self._seen_sequences.items() if timestamp >= cutoff
                    }
            if sequence is None and now - self.last_press_time < self.debounce_seconds:
                continue
            self.last_press_time = now
            self.press_count += 1
            print(
                f"[OpenArmTaskStudio] Remote record button #{self.press_count} from "
                f"{address[0]}:{address[1]}"
            )
        return self.press_count

    def status(self):
        return {
            "last_packet_time": self.last_packet_time,
            "last_press_time": self.last_button_time,
            "address": self.last_address,
            "is_down": self.button_is_down,
            "sequence": self.last_sequence,
        }

    @staticmethod
    def _is_record_message(data):
        return RemoteRecordButtonReceiver._record_sequence(data) is not False

    @staticmethod
    def _record_sequence(data):
        try:
            text = data.decode("utf-8").strip()
        except UnicodeDecodeError:
            return False
        if text.upper() in {"CONFIRM", "RECORD", "BUTTON", "1"}:
            return None
        prefix, separator, value = text.upper().partition(":")
        if separator and prefix == "RECORD" and value.isdigit():
            return int(value)
        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
            return False
        if str(payload.get("action", "")).lower() not in {"confirm", "record"}:
            return False
        sequence = payload.get("sequence")
        if sequence is None:
            return None
        try:
            return int(sequence)
        except (TypeError, ValueError):
            return False

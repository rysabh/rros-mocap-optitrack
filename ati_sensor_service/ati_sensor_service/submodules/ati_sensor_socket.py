import errno
import socket
import struct
import threading
import time

import numpy as np


class NetFTSensor:
    def __init__(self, ip: str, port: int = 49152):
        self.ip = ip
        self.port = port
        self.sock = None
        self.latest_data = None
        self.start_command = b"\x12\x34\x00\x02\x00\x00\x00\x00"
        self.stop_command = b"\x12\x34\x00\x00\x00\x00\x00\x01"

        self.running = False
        self.data_thread = None

    def send_command(self, command: bytes) -> None:
        self.sock.sendto(command, (self.ip, self.port))

    def start_streaming(self) -> None:
        if self.running:
            print("ATI sensor streaming is already running.")
            return

        print("Starting ATI sensor stream...")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.connect((self.ip, self.port))
        self.send_command(self.start_command)

        self.running = True
        self.data_thread = threading.Thread(target=self._update_data, daemon=True)
        self.data_thread.start()

    def stop_streaming(self) -> None:
        if not self.running:
            return

        print("Stopping ATI sensor stream...")
        self.running = False
        self.send_command(self.stop_command)

        if self.data_thread:
            self.data_thread.join()
            self.data_thread = None

        if self.sock:
            self.sock.close()
            self.sock = None
        print("ATI sensor socket closed.")

    def _update_data(self) -> None:
        while self.running:
            try:
                data, _ = self.sock.recvfrom(36)
                if data:
                    self.latest_data = NetFTRDTPacket(data)
            except socket.error as e:
                if e.errno not in (errno.EWOULDBLOCK, errno.EAGAIN):
                    print(f"ATI sensor socket error: {e}")

    def get_most_recent_data(self):
        return self.latest_data if self.latest_data else "No data received yet."


class NetFTRDTPacket:
    def __init__(self, data: bytes):
        self.data = data
        self.force_torque_data = np.zeros(6)
        self.ft_timestamp_ms = 0
        self._unpack_data()

    def _unpack_data(self) -> None:
        fx_b = bytearray(self.data[12:16])
        fy_b = bytearray(self.data[16:20])
        fz_b = bytearray(self.data[20:24])
        tx_b = bytearray(self.data[24:28])
        ty_b = bytearray(self.data[28:32])
        tz_b = bytearray(self.data[32:36])

        fx = struct.unpack("!i", fx_b)[0] / 1e6
        fy = struct.unpack("!i", fy_b)[0] / 1e6
        fz = struct.unpack("!i", fz_b)[0] / 1e6
        tx = struct.unpack("!i", tx_b)[0] / 1e6
        ty = struct.unpack("!i", ty_b)[0] / 1e6
        tz = struct.unpack("!i", tz_b)[0] / 1e6

        self.force_torque_data = np.array([fx, fy, fz, tx, ty, tz])
        self.ft_timestamp_ms = int(time.time() * 1000)

    def get_force_torque_array(self):
        return self.force_torque_data

    def get_ft_timestamp(self) -> float:
        return self.ft_timestamp_ms * 1e-3


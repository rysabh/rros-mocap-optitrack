import socket
import struct
import numpy as np
import time
import errno
import threading

class NetFTSensor:
    def __init__(self, ip, port=49152):
        self.ip = ip
        self.port = port
        self.sock = None
        self.latest_data = None  # To store the latest data
        self.start_command = b'\x12\x34\x00\x02\x00\x00\x00\x00'
        self.stop_command = b'\x12\x34\x00\x00\x00\x00\x00\x01'

        # THREADING
        self.running = False  # Flag to check if data stream is running
        self.data_thread = None  # Thread to continuously keep the data being updated in parallel

    def send_command(self, command):
        """Function for sending any commands."""
        self.sock.sendto(command, (self.ip, self.port))

    def start_streaming(self):
        """Command to start streaming the data."""

        if not self.running:
            print("Starting ATI SENSOR stream...")
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.connect((self.ip, self.port))
            self.send_command(self.start_command)
            self.running = True
            self.data_thread = threading.Thread(target=self._update_data)
            self.data_thread.start()
        else:
            print("ATI SENSOR Streaming is already running.")

    def stop_streaming(self):
        """Command to stop streaming the data."""

        if self.running:
            print("Stopping streaming of ATI SENSOR...")
            self.running = False
            self.send_command(self.stop_command)
            if self.data_thread:
                self.data_thread.join()
            if self.sock:
                self.sock.close()
            print("Socket closed to ATI SENSOR.")


    def _update_data(self):
        """Background thread function to update sensor data continuously."""

        while self.running:
            try:
                data, _ = self.sock.recvfrom(36)
                if data:
                    self.latest_data = NetFTRDTPacket(data)
            except socket.error as e:
                if e.errno not in (errno.EWOULDBLOCK, errno.EAGAIN):
                    print(f"Socket error: {e}")

    def get_most_recent_data(self):
        """Return the most recently received data packet, or a status message if no data."""
        return self.latest_data if self.latest_data else "No data received yet."

class NetFTRDTPacket:
    def __init__(self, data):
        self.data = data
        self.unpack_data()

    def unpack_data(self):

        # SPACE 
        _Fx_ = bytearray(4)
        _Fy_ = bytearray(4)
        _Fz_ = bytearray(4)
        _Tx_ = bytearray(4)
        _Ty_ = bytearray(4)
        _Tz_ = bytearray(4)

        # EXTRACT
        for i in range(4):
            _Fx_[i] = self.data[12 + i]
            _Fy_[i] = self.data[16 + i]
            _Fz_[i] = self.data[20 + i]
            _Tx_[i] = self.data[24 + i]
            _Ty_[i] = self.data[28 + i]
            _Tz_[i] = self.data[32 + i]

        # UNPACK AND DIVIDE BY COUNTS
        data_Fx = struct.unpack('!i', _Fx_)[0] / 1e6
        data_Fy = struct.unpack('!i', _Fy_)[0] / 1e6
        data_Fz = struct.unpack('!i', _Fz_)[0] / 1e6
        data_Tx = struct.unpack('!i', _Tx_)[0] / 1e6
        data_Ty = struct.unpack('!i', _Ty_)[0] / 1e6
        data_Tz = struct.unpack('!i', _Tz_)[0] / 1e6

        self.force_torque_data = np.array([data_Fx, data_Fy, data_Fz, data_Tx, data_Ty, data_Tz])
        self.ft_timestamp = int(time.time() * 1000)  # SAME AS CAMERA RECORDER

        # unpacked_data = struct.unpack('!IIIIIIIII', self.data)
        # self.rdt_sequence = unpacked_data[0]
        # self.ft_sequence = unpacked_data[1]
        # self.status = unpacked_data[2]
        # self.force_torque_data = np.array(unpacked_data[3:])

    def get_force_torque_array(self):
        """Convert force and torque data to a numpy array for easy handling."""
        return self.force_torque_data
    
    def get_ft_timestamp(self):
        return self.ft_timestamp*1e-3



# Example usage
if __name__ == "__main__":
    sensor = NetFTSensor("192.168.10.100")  # IP Address of the sensor
    sensor.start_streaming()
    time.sleep(0.002)

    start_packet = sensor.get_most_recent_data()
    start_time = start_packet.get_ft_timestamp()


    try:
        while True:
            time.sleep(0.002)  # Polling delay to match the sensor's publishing rate
            packet = sensor.get_most_recent_data()
            
            if packet:
                print(packet.force_torque_data)
                time_elapsed = packet.get_ft_timestamp() - start_time
                print(time_elapsed)
            else:
                print(packet)

    except KeyboardInterrupt:
        print("\nStreaming interrupted by user.")
    finally:
        sensor.stop_streaming()


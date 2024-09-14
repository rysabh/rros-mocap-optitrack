import rclpy
from rclpy.node import Node
import csv
import time
from data_collection.submodules.ati_sensor_socket import NetFTSensor
from ati_sensor_interfaces.msg import ForceTorque

class NetFTSensorPublisher(Node):
    def __init__(self):
        super().__init__('netft_sensor_writer')

        self.declare_parameter('sensor_ip', '192.168.10.100')
        self.declare_parameter('output_file', 'force_data.csv')
        self.declare_parameter('sample_rate', '1')

        sensor_ip = self.get_parameter('sensor_ip').get_parameter_value().string_value
        output_file = self.get_parameter('output_file').get_parameter_value().string_value
        sample_rate = float(self.get_parameter('sample_rate').get_parameter_value().string_value)

        self.sensor = NetFTSensor(sensor_ip)
        self.sensor.start_streaming()
        time.sleep(1 / sample_rate)

        self.csv_file = open(output_file, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time Elapsed (s)', 'Fx (N)', 'Fy (N)', 'Fz (N)', 'Tx (Nm)', 'Ty (Nm)', 'Tz (Nm)'])

        start_packet = self.sensor.get_most_recent_data()
        
        # Check if start_packet is valid and has the necessary method
        if isinstance(start_packet, str) or not hasattr(start_packet, 'get_ft_timestamp'):
            self.get_logger().error("Failed to receive valid data from the sensor. Retrying...")
            self.start_time = time.time()  # Fallback to current time if no valid timestamp is received
        else:
            self.start_time = start_packet.get_ft_timestamp()

        self.publisher_ = self.create_publisher(ForceTorque, 'force_torque', 10)
        self.timer = self.create_timer(1 / sample_rate, self.record_data)

    def record_data(self):
        packet = self.sensor.get_most_recent_data()

        if packet and isinstance(packet, object) and hasattr(packet, 'get_ft_timestamp'):
            time_elapsed = packet.get_ft_timestamp() - self.start_time
            ft_data = packet.get_force_torque_array()
            self.csv_writer.writerow([time_elapsed, *ft_data])
            self.get_logger().info(f"\nTime: {round(time_elapsed, 2)}s, Data: {[round(i, 2) for i in ft_data]}")

            msg = ForceTorque()
            msg.time_elapsed = time_elapsed
            msg.fx = ft_data[0]
            msg.fy = ft_data[1]
            msg.fz = ft_data[2]
            msg.tx = ft_data[3]
            msg.ty = ft_data[4]
            msg.tz = ft_data[5]
            self.publisher_.publish(msg)
        else:
            self.get_logger().warning("No valid data received from the sensor.")

    def destroy_node(self):
        self.sensor.stop_streaming()
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = NetFTSensorPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

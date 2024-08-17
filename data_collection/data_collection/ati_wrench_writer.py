import rclpy
from rclpy.node import Node
import csv
import time
from data_collection.submodules.ati_sensor_socket import NetFTSensor


class NetFTSensorWriter(Node):
    def __init__(self):
        super().__init__('netft_sensor_writer') # node name

        self.declare_parameter('sensor_ip', '192.168.10.100')
        self.declare_parameter('output_file', 'force_data.csv')
        self.declare_parameter('sample_rate', '1')
        
        sensor_ip = self.get_parameter('sensor_ip').get_parameter_value().string_value
        output_file = self.get_parameter('output_file').get_parameter_value().string_value
        sample_rate = float(self.get_parameter('sample_rate').get_parameter_value().string_value)

        self.sensor = NetFTSensor(sensor_ip)
        self.sensor.start_streaming()
        time.sleep(1/sample_rate)

        self.csv_file = open(output_file, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time Elapsed (s)', 'Fx (N)', 'Fy (N)', 'Fz (N)', 'Tx (Nm)', 'Ty (Nm)', 'Tz (Nm)'])

        start_packet = self.sensor.get_most_recent_data()
        self.start_time = start_packet.get_ft_timestamp()

        self.timer = self.create_timer(1/sample_rate, self.record_data)

    def record_data(self):
        packet = self.sensor.get_most_recent_data()

        if packet and packet != "No data received yet.":
            time_elapsed = packet.get_ft_timestamp() - self.start_time
            ft_data = packet.get_force_torque_array()
            self.csv_writer.writerow([time_elapsed, *ft_data])
            self.get_logger().info(f"\nTime: {round(time_elapsed,2)}s, Data: {[round(i,2) for i in ft_data]}")
    
    def destroy_node(self):
        self.sensor.stop_streaming()
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = NetFTSensorWriter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
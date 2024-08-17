import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import numpy as np

import csv
from std_msgs.msg import String  # Replace with the appropriate message type


class WrenchSubscriberNode(Node):
    WRENCH_TOPIC = "/lbr/force_torque_broadcaster/wrench"

    def __init__(self) -> None:
        super().__init__("wrench_subscriber")

        self.latest_wrench = None

        self.wrench_sub = self.create_subscription(
            WrenchStamped,
            self.WRENCH_TOPIC,
            self.wrench_callback,
            10
        )
        
        # #---- save to csv
        # self.csv_file = open('kuka_wrench_data_stream.csv', mode='w')
        # self.csv_writer = csv.writer(self.csv_file)
        # #---- save to csv

    def wrench_callback(self, msg):
        self.latest_wrench = self.convert_to_numpy(msg)
        # self.get_logger().info(f"Received wrench data: {self.latest_wrench}")
     
        # #---- save to csv
        # fx = msg.wrench.force.x
        # fy = msg.wrench.force.y
        # fz = msg.wrench.force.z
        # tx = msg.wrench.force.x
        # ty = msg.wrench.force.y
        # tz = msg.wrench.force.z
        # self.csv_writer.writerow([fx,fy,fz,tx,ty,tz]) # Append data to the CSV file
        # #---- save to csv

    def convert_to_numpy(self, msg):
        force = msg.wrench.force
        torque = msg.wrench.torque
        return np.array([force.x, force.y, force.z, torque.x, torque.y, torque.z])
    
    def get_latest_kuka_wrench(self):
        return self.latest_wrench

def main(args=None):
    rclpy.init(args=args)
    wrench_sub = WrenchSubscriberNode()
    rclpy.spin(wrench_sub)

    # #---- save to csv
    # rclpy.csv_file.close()
    # #---- save to csv

    wrench_sub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

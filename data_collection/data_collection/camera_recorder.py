import os
import time

import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge

import sensor_msgs


class CameraRecorderNode(Node):
    RGB_TOPIC = "/camera/camera/color/image_raw"
    DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"

    def __init__(self) -> None:
        super().__init__("camera_recorder")

        self.bridge = CvBridge()

        self.rgb_buffer = []
        self.rgb_buffer_size = 10

        self.rgb_sub = self.create_subscription(
            sensor_msgs.msg.Image,
            self.RGB_TOPIC,
            self.rgb_callback,
            self.rgb_buffer_size,
        )

    def rgb_callback(self, msg):
        while len(self.rgb_buffer) >= self.rgb_buffer_size:
            self.rgb_buffer.pop(0)

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.rgb_buffer.append(cv_image)

    def camera_record(self, save_dir) -> str:
        timestamp = int(time.time() * 1000)
        rgb_msg = self.rgb_buffer[-1]

        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        # self.get_logger().info(f"Saving image to {save_dir}")

        cv2.imwrite(f"{save_dir}/{timestamp}_rgb.png", rgb_msg)

        return str(timestamp)


def main(args=None):
    rclpy.init(args=args)
    camera_recorder = CameraRecorderNode()
    rclpy.spin(camera_recorder)

    camera_recorder.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

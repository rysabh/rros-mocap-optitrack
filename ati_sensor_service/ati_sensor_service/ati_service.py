import time
from typing import Optional

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.time import Time

from ati_sensor_interfaces.msg import ForceTorque
from ati_sensor_interfaces.srv import GetForceTorque

from ati_sensor_service.submodules.ati_sensor_socket import NetFTSensor


class AtiSensorNode(Node):
    def __init__(self) -> None:
        super().__init__("ati_sensor")

        self.declare_parameter("sensor_ip", "192.168.10.100")
        self.declare_parameter("sample_rate", 240.0)
        self.declare_parameter("frame_id", "ati_sensor")

        sensor_ip = self.get_parameter("sensor_ip").get_parameter_value().string_value
        sample_rate_raw = self.get_parameter("sample_rate").value
        sample_rate_hz = float(sample_rate_raw)
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        self._frame_id = frame_id
        self._latest_force_msg: Optional[ForceTorque] = None
        self._last_no_data_log_time_ns = 0

        self._sensor = NetFTSensor(sensor_ip)
        self._sensor.start_streaming()
        time.sleep(max(0.001, 1.0 / max(sample_rate_hz, 1.0)))

        self._force_pub = self.create_publisher(ForceTorque, "force_torque", 10)
        self._wrench_pub = self.create_publisher(WrenchStamped, "wrench", 10)
        self._get_force_srv = self.create_service(
            GetForceTorque, "get_force_torque", self._handle_get_force_torque
        )

        self._timer = self.create_timer(1.0 / sample_rate_hz, self._publish_once)

    def _handle_get_force_torque(
        self, request: GetForceTorque.Request, response: GetForceTorque.Response
    ) -> GetForceTorque.Response:
        if self._latest_force_msg is not None:
            response.msg = self._latest_force_msg
        return response

    def _publish_once(self) -> None:
        packet = self._sensor.get_most_recent_data()
        if isinstance(packet, str) or not hasattr(packet, "get_ft_timestamp"):
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_no_data_log_time_ns > int(5e9):
                self.get_logger().warning("No valid data received from the ATI sensor yet.")
                self._last_no_data_log_time_ns = now_ns
            return

        timestamp_s = float(packet.get_ft_timestamp())
        fx, fy, fz, tx, ty, tz = [float(v) for v in packet.get_force_torque_array()]

        force_msg = ForceTorque()
        force_msg.time_elapsed = timestamp_s
        force_msg.fx = fx
        force_msg.fy = fy
        force_msg.fz = fz
        force_msg.tx = tx
        force_msg.ty = ty
        force_msg.tz = tz
        self._latest_force_msg = force_msg

        self._force_pub.publish(force_msg)

        stamp_ns = int(timestamp_s * 1e9)
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
        wrench_msg.header.frame_id = self._frame_id
        wrench_msg.wrench.force.x = fx
        wrench_msg.wrench.force.y = fy
        wrench_msg.wrench.force.z = fz
        wrench_msg.wrench.torque.x = tx
        wrench_msg.wrench.torque.y = ty
        wrench_msg.wrench.torque.z = tz
        self._wrench_pub.publish(wrench_msg)

    def destroy_node(self) -> bool:
        self._sensor.stop_streaming()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AtiSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
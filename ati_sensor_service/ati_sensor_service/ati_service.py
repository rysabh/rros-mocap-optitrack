import rclpy
from rclpy.node import Node
from ati_sensor_interfaces.msg import ForceTorque
from ati_sensor_interfaces.srv import GetForceTorque

class AtiService(Node):

    def __init__(self):
        super().__init__('ati_service')
        self.service_ = self.create_service(GetForceTorque, 'get_force_torque', self.get_force_torque_callback)
        self.latest_message = None
        self.subscription = self.create_subscription(ForceTorque, 'force_torque', self.listener_callback, 10)

    def get_force_torque_callback(self, request, response):
        if self.latest_message is not None:
            response.msg = self.latest_message
        return response

    def listener_callback(self, msg):
        self.latest_message = msg

    def destroy_node(self):
        self.subscription.destroy()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    ati_service = AtiService()
    rclpy.spin(ati_service)
    ati_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
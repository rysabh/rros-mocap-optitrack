import rclpy
from rclpy.node import Node
from mocap_optitrack_interfaces.srv import GetMotionCaptureData
from mocap_optitrack_interfaces.msg import MotionCaptureData

class MocapServiceSubscriber(Node):
    def __init__(self):
        super().__init__('mocap_service_subscriber')
        self.service = self.create_service(
            GetMotionCaptureData,
            'get_mocap_data',
            self.handle_service_request)
        self.latest_message = None
        self.subscription = self.create_subscription(
            MotionCaptureData,
            'mocap_Data',
            self.listener_callback,
            10)
        
    def handle_service_request(self, request, response):
        if self.latest_message:
            response.latest_message = self.latest_message
        return response
    
    def listener_callback(self, msg):
        self.latest_message = msg

    def destroy_node(self):
        self.subscription.destroy()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    mocap_service_subscriber = MocapServiceSubscriber()
    rclpy.spin(mocap_service_subscriber)
    mocap_service_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
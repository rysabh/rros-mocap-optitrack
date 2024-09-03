import rclpy
from rclpy.node import Node
from mocap_optitrack_interfaces.srv import GetMotionCaptureData

class MocapClient(Node):
    def __init__(self):
        super().__init__('mocap_client')
        self.client = self.create_client(GetMotionCaptureData, 'get_mocap_data')
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('service not available, waiting again...')

    def send_request(self):
        request = GetMotionCaptureData.Request()
        future = self.client.call_async(request)
        return future

def main(args=None):
    rclpy.init(args=args)
    mocap_client = MocapClient()
    future = mocap_client.send_request()
    rclpy.spin_until_future_complete(mocap_client, future)

    if future.done():
        try:
            response = future.result()
            mocap_client.get_logger().info('Service call completed')
            print("Received mocap data: ", response.latest_message)
        except KeyboardInterrupt:
            pass
        finally:
            mocap_client.destroy_node()
            rclpy.shutdown()



if __name__ == '__main__':
    main()
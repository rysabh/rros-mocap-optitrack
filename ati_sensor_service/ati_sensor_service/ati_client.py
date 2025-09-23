import rclpy
from rclpy.node import Node
from ati_sensor_interfaces.srv import GetForceTorque

class AtiClient(Node):

    def __init__(self):
        super().__init__('ati_client')
        self.client_ = self.create_client(GetForceTorque, 'get_force_torque')
        while not self.client_.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('service not available, waiting again...')

    def get_force_torque(self):
        request = GetForceTorque.Request()
        future = self.client_.call_async(request)

        return future
        # Call the service synchronously and wait for the response
        # response = self.client_.call(request)
        
        # return response
                # Attempt to call the service
        # try:
        #     print("Calling service")
        #     response = self.client_.call(request)
        #     self.get_logger().info(f"Received response: {response}")
        #     return response
        # except Exception as e:
        #     self.get_logger().error(f"Failed to call service: {str(e)}")
        #     return None
    
def main(args=None):
    rclpy.init(args=args)
    ati_client = AtiClient()
    future = ati_client.get_force_torque()
    rclpy.spin_until_future_complete(ati_client, future)

    if future.done():
        try:
            response = future.result()
            ati_client.get_logger().info(f"Service call successful")
            print(response.msg)
        except KeyboardInterrupt:
            pass
        finally:
            ati_client.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from mocap_optitrack_interfaces.msg import RigidBodyArray
from geometry_msgs.msg import PoseStamped
from 

class KukaPosePub(Node):
    def __init__(self):
        super().__init__('kuka_pose_pub')
        self.publisher_ = self.create_publisher(PoseStamped, 'kuka_blue_pose', 10)
        self.subscription = self.create_subscription(RigidBodyArray, 'mocap_rigid_bodies', self.callback, 10)
        self.subscription  # prevent unused variable warning

    def callback(self, msg):
        for body in msg.rigid_bodies:
            if body.id == 16:
                pose_stamped = PoseStamped()
                pose_stamped = body.pose_stamped
                pose_list = [pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z,
                             pose_stamped.pose.orientation.w, pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y,
                             pose_stamped.pose.orientation.z]
                print(pose_list)
                self.publisher_.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    kuka_pose_pub = KukaPosePub()
    rclpy.spin(kuka_pose_pub)
    kuka_pose_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
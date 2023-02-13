import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
class Driver_node(Node):

    def __init__(self):
        super().__init__('driving_custom_Node')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(1, self.timer_callback)



    def timer_callback(self):
        msg = Twist()
        # W = v / r
        linear_vel= float(sys.argv[1])
        radius = float(sys.argv[2])
        msg.linear.x=linear_vel
        msg.linear.y= 0.0
        msg.angular.z=linear_vel/radius
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Driver_node()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
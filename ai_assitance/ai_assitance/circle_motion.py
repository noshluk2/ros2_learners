
import rclpy
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist

class CircleMovement:

    def __init__(self):
        self.node = rclpy.create_node('circle_movement')
        qos = QoSProfile(depth=10)
        self.pub = self.node.create_publisher(Twist, 'cmd_vel',qos)
        self.timer = self.node.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.9
        self.pub.publish(msg)

    def spin(self):
        rclpy.spin(self.node)

def main(args=None):
    rclpy.init(args=args)
    circle_movement = CircleMovement()
    circle_movement.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

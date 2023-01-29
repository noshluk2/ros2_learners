import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        qos = rclpy.qos.QoSProfile(depth=10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', qos)
        self.obstacle_detected = False

    def scan_callback(self, msg):
        # Process laser scan data to detect obstacles
        for i in range(len(msg.ranges)):
            if msg.ranges[i] < 0.5: # obstacle is detected if range is less than 0.5m
                self.obstacle_detected = True
                break
        if self.obstacle_detected:
            self.avoid_obstacle()
        else:
            self.stop_robot()

    def avoid_obstacle(self):
        # Publish Twist message to move robot in a circular motion
        twist_msg = Twist()
        twist_msg.linear.x = 0.1
        twist_msg.angular.z = 0.7
        self.publisher.publish(twist_msg)

    def stop_robot(self):
        # Publish Twist message to stop robot
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)

if __name__ == '__main__':
    main()

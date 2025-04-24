#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import statistics
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
class DrivingNode(Node):

    def __init__(self):
        super().__init__('driving_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.subscriber = self.create_subscription(LaserScan,'scan',self.listener_callback,10)
        
    def listener_callback(self,msg):
        range = msg.ranges[350:370]
        median = statistics.median(range)
        movement_msg = Twist()
        if (median > 1000 or median < 0.4):
            print("NOT DRIVING")
            print(median)
            movement_msg.linear.x = 0.0
            movement_msg.linear.y = 0.0
            movement_msg.linear.z = 0.0
            movement_msg.angular.x = 0.0
            movement_msg.angular.y = 0.0
            movement_msg.angular.z = 0.0
        else:
            print("DRIVING")
            movement_msg.linear.x = 0.1
            movement_msg.linear.y = 0.0
            movement_msg.linear.z = 0.0
            movement_msg.angular.x = 0.0
            movement_msg.angular.y = 0.0
            movement_msg.angular.z = 0.0
        self.publisher.publish(movement_msg)

    

def main(args=None):
    rclpy.init(args=args)

    driving_node = DrivingNode()

    rclpy.spin(driving_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    driving_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
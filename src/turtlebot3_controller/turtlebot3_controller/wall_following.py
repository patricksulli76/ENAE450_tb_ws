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
        self.following_right = False
        self.following_left = False
        self.turning_right = False
        self.turning_left = False
        self.speed = 0.1
        self.distance = 0.5
        self.turn_speed = 1.0
        self.crit_distance = 0.3

    def turn_left(self):
        movement_msg = Twist()
        movement_msg.linear.x = 0.0
        movement_msg.linear.y = 0.0
        movement_msg.linear.z = 0.0
        movement_msg.angular.x = 0.0
        movement_msg.angular.y = 0.0
        movement_msg.angular.z = self.turn_speed
        return movement_msg
    
    def turn_right(self):
        movement_msg = Twist()
        movement_msg.linear.x = 0.0
        movement_msg.linear.y = 0.0
        movement_msg.linear.z = 0.0
        movement_msg.angular.x = 0.0
        movement_msg.angular.y = 0.0
        movement_msg.angular.z = -self.turn_speed
        return movement_msg
    
    def move_forward(self):
        movement_msg = Twist()
        movement_msg.linear.x = self.speed
        movement_msg.linear.y = 0.0
        movement_msg.linear.z = 0.0
        movement_msg.angular.x = 0.0
        movement_msg.angular.y = 0.0
        movement_msg.angular.z = 0.0
        return movement_msg
    
    def move_backward(self):
        movement_msg = Twist()
        movement_msg.linear.x = -self.speed
        movement_msg.linear.y = 0.0
        movement_msg.linear.z = 0.0
        movement_msg.angular.x = 0.0
        movement_msg.angular.y = 0.0
        movement_msg.angular.z = 0.0
        return movement_msg
    

    
    def stop(self):
        movement_msg = Twist()
        movement_msg.linear.x = 0.0
        movement_msg.linear.y = 0.0
        movement_msg.linear.z = 0.0
        movement_msg.angular.x = 0.0
        movement_msg.angular.y = 0.0
        movement_msg.angular.z = 0.0
        return movement_msg
    
    def check_wall_in_front(self, msg, distance):
        range = msg.ranges[350:370]
        median = statistics.median(range)
        if (median < distance):
            return True
        else:
            return False
    
    def check_wall_on_left(self, msg, distance):
        range = msg.ranges[85*2:95*2]
        median = statistics.median(range)
        if (median < distance):
            return True
        else:
            return False
    def check_wall_on_right(self, msg, distance):
        range = msg.ranges[265*2:275*2]
        median = statistics.median(range)
        if (median < distance):
            return True
        else:
            return False
    def check_wall_on_back(self, msg, distance):
        range = msg.ranges[355*2:365*2]
        median = statistics.median(range)
        if (median < distance):
            return True
        else:
            return False
    
    def check_wall_all(self, msg, distance):
        range = msg.ranges[0:720]
        min = min(range)
        if (min < distance):
            return True
        else:
            return False
    """
    try:
            if self.following_right:
                if (self.check_wall_on_right):
                    movement_msg = self.move_forward()
                    print("DRIVING")

                else:
                    print("TURNING RIGHT")
                    movement_msg = self.turn_right
            elif self.following_left:
                if (self.check_wall_on_left):
                    movement_msg = self.move_forward()
                    print("DRIVING")
                else:
                    movement_msg = self.turn_left
                    print("TURNING LEFT")
            else:
                print("Inital state")
                if (not self.check_wall_in_front(msg, self.distance)):
                    print("DRIVING")
                    print("Wall: %f", self.check_wall_in_front(msg, self.distance))
                    movement_msg = self.move_forward()
                elif (self.check_wall_on_left(msg, self.distance)):
                    print("TURNING RIGHT")
                    movement_msg = self.turn_right()
                    self.following_right = True
                elif (self.check_wall_on_right(msg, self.distance)):
                    print("TURNING LEFT")
                    movement_msg = self.turn_left()
                    self.following_left = True
        except:
            print("Error in listener_callback
    """
    def listener_callback(self,msg):
        movement_msg = Twist()
        movement_msg = self.stop()
        if self.following_right:
            print("Following right")
            if(self.check_wall_all(msg, self.crit_distance)):
                self.movement_msg = self.move_backward()
                self.movement_msg.angular.z = self.turn_speed
            elif (self.check_wall_on_right and not self.check_wall_in_front(msg, self.distance)):
                movement_msg = self.move_forward()
                print("DRIVING")
            else:
                print("TURNING Left")
                movement_msg = self.turn_right()
        elif self.following_left:
            print("Following left")
            if(self.check_wall_in_front(msg, self.distance)):
                print("TURNING RIGHT")
                movement_msg = self.turn_right()
                self.following_right = True
            elif (self.check_wall_on_right and not self.check_wall_in_front(msg, self.distance)):
                movement_msg = self.move_forward()
                print("DRIVING")
            else:
                print("TURNING Right")
                movement_msg = self.turn_right()
        else:
            print("Inital state")
            if (not self.check_wall_in_front(msg, self.distance)):
                print("DRIVING")
                print("Wall: %f", self.check_wall_in_front(msg, self.distance))
                movement_msg = self.move_forward()
            
            elif (not self.check_wall_on_right(msg, self.distance)):
                print("TURNING LEFT")
                movement_msg = self.turn_left()
            elif (not self.check_wall_on_left(msg, self.distance)):
                print("TURNING RIGHT")
                movement_msg = self.turn_right()

            if (self.check_wall_on_right(msg, self.distance)):
                self.following_right = True
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
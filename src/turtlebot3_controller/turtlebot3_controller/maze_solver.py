#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import statistics
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
import numpy as np
class DrivingNode(Node):

    def __init__(self):
        super().__init__('driving_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.subscriber = self.create_subscription(LaserScan,'scan',self.listener_callback,10)
        self.following_right = True
        self.following_left = False
        self.turning_right = False
        self.turning_left = False
        self.speed = 0.1
        self.distance = 0.5
        self.turn_speed = 1.0
        self.crit_distance = 0.3

        self.servo = False

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
        minimum = min(range)
        if (minimum < distance):
            return True
        else:
            return False
        
    def get_max_direction(self, msg):
        """
        Get the maximum distance in the range of 0 to 720 degrees
        """
        ranges = [0 if x == float('inf') else x for x in msg.ranges[0:720]]
        max_distance = max(ranges[360-180:360+180])
        max_index = ranges.index(max_distance)
        return max_index, max_distance
    
    def check_wall_side(self, msg, offset,tolarance):
        big_number = 1000
        ranges = [10 if x == float('inf') else x for x in msg.ranges[0:720]]
        direction = 360
        left_avg = np.median(ranges[direction-offset:direction])
        right_avg = np.median(ranges[direction:direction+offset])
        front_avg = np.median(ranges[direction-int(offset/2):direction+int(offset/2)])
        closest = min(left_avg, right_avg, front_avg)
        print("Left: %f, Right: %f, Front: %f" % (left_avg, right_avg, front_avg))

        if (closest < tolarance):
            if closest == left_avg:
                return "WALL ON LEFT"
            elif closest == right_avg:
                return "WALL ON RIGHT"
            else:
                return "WALL IN FRONT"

        else:
            return "NO WALL"
        




        
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
        print("Length of ranges: %d" % len(msg.ranges))
        movement_msg = Twist()
        direction, distance = self.get_max_direction(msg)
        offset = 10
        delay = 1

        wall_offset = 120

        wall_side = self.check_wall_side(msg,  wall_offset, 0.3)
        print("Wall side: %s" % wall_side)

        if wall_side != "NO WALL":
            delay = 500

        if(self.servo):
            print("STOPPING")
            movement_msg = self.stop()
            self.servo = False
        else:
            if direction in range(360-offset, 360+offset) and wall_side == "NO WALL":
                print("DRIVING")
                movement_msg = self.move_forward()
            elif wall_side == "WALL IN FRONT":
                print("Moving Backward")
                movement_msg = self.move_backward()
                delay *= 4
                
            elif (direction - 360) > 0 or wall_side == "WALL ON RIGHT":
                print("TURNING LEFT")
                movement_msg = self.turn_left()
                self.servo = True
            elif (direction - 360) < 0 or wall_side == "WALL ON LEFT":
                print("TURNING RIGHT")
                movement_msg = self.turn_right()
                self.servo = True
            else:
                print("STOPPING")
                movement_msg = self.stop()
            
        if wall_side != "NO WALL":
            movement_msg.linear.x = -self.speed

        self.publisher.publish(movement_msg)
        time.sleep(delay/1000)
        
        


    

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
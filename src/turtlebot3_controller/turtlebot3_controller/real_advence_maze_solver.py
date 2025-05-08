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

        self.x = 0
        self.y = 0
        self.orientation = [0,1]

        self.translation_speed = 0.1
        self.rotation_speed = 0.1

        self.last_laser_scan = None

        self.warning_distance = 0.4
        self.stop_distance = 0.3

    def max_distances(self, msg):
        """
        Return sorted distances from max to min in a given range with their corresponding indices
        Parameters:
            msg: LaserScan message containing range data
        Returns:
            A list of tuples (distance, index) sorted by distance in descending order
        """
        ranges = msg.ranges[80*2:280*2]  # Get the range data from 90° to 270°
        # Create index-value pairs, treating inf as a large number
        valid_pairs = []
        for i, dist in enumerate(ranges):
            if np.isnan(dist):
                continue  # Skip NaN values
            elif dist == float('inf'):
                # Treat infinity as a large number (e.g., 100 meters)
                valid_pairs.append((i+80*2, 100.0))
            else:
                valid_pairs.append((i+80*2, dist))
        # Sort by distance (descending)
        sorted_pairs = sorted(valid_pairs, key=lambda x: x[1], reverse=True)
        return sorted_pairs
    
    def follow_max(self, msg, movement_msg, max_distance):
        """
        Make the robot follow the maximum distance path
        Parameters:
            msg: LaserScan message
            movment_msg: Twist message to fill
            max_distances: List of (index, distance) tuples sorted by distance (descending)
        """
        # Check if we have valid distance readings
        if not max_distance:
            # No valid readings, stop
            movement_msg.linear.x = 0.0
            movement_msg.angular.z = 0.0
            return

        # Get the index and distance of the furthest point
        max_index, max_dist = max_distance
        center_index = 180*2 # The center of our scan range (straight ahead)

        # Define angle threshold for considering a path "straight ahead"
        angle_threshold = 1 # degrees

        # Calculate the angular difference from center
        angle_diff = abs(max_index - center_index)

        if angle_diff <= angle_threshold:
            # The max distance is roughly straight ahead, move forward
            movement_msg.linear.x = self.translation_speed
            movement_msg.angular.z = 0.0
        else:
            # Max distance is not straight ahead, stop and turn towards it
            if self.check_obstacle(msg): 
                movement_msg.linear.x = 0.0
            else:
                movement_msg.linear.x = self.translation_speed
            
            # Determine turn direction
            if max_index < center_index:
                # Turn left
                movement_msg.angular.z = self.rotation_speed *abs(max_index - center_index)/10
                print("Turning left")
            else:
                # Turn right
                movement_msg.angular.z = -self.rotation_speed *abs(max_index - center_index)/10
                print("Turning right")
        
        return movement_msg
    
    def check_obstacle(self, msg):
        """
        Check if there is an obstacle in front of the robot
        Parameters:
            msg: LaserScan message
        Returns:
            True if an obstacle is detected, False otherwise
        """
        # Get the distance at the front (index 0)
        front_distance = msg.ranges[0]
        
        for i in range(135*2, 225*2):
                r = msg.ranges[i]
                # Check if this is a valid reading and closer than warning distance
                if r < self.stop_distance:
                    print(f"Obstacle found at index {i} with distance {r}")
                    obstacle_found = True
                    return True
        
        return False
    
    def check_viable_max(self, max_distances, msg):
        """
        Check for the first pair in max_distances that has no obstacles
        within +/- 45 degrees that are closer than warning distance
        
        Parameters:
            max_distances: List of (index, distance) tuples sorted by distance
            msg: LaserScan message containing range data
        
        Returns:
            Tuple (index, distance) if found, None otherwise
        """
        if not max_distances:
            return None
        
        for idx, dist in max_distances:

            # Define the range to check (+/- 45 degrees)
            start_idx = max(idx-40*2,0)
            end_idx = min(idx+40*2, len(msg.ranges)-1)
            
            # Check if there are any obstacles closer than warning distance
            obstacle_found = False
            for i in range(int(start_idx), int(end_idx) + 1):
                r = msg.ranges[i]
                # Check if this is a valid reading and closer than warning distance
                if r < self.warning_distance:
                    print(f"Obstacle found at index {i} with distance {r}")
                    obstacle_found = True
                    break

            if not obstacle_found:
                return (idx, dist)
        
        return None

    
    def sim_to_real(self,msg):
        """
        Convert a 360‑element (1° per step, front at 0°) scan to a
        720‑element (0.5° per step, front at index 360) scan.

        ── How it works ─────────────────────────────────────────────
        • Each 1° measurement is duplicated so we now have 0.5° spacing.
        • Angles are rotated +180° so that “front” lands at index 360.
        Mapping formula: real_idx = ((deg + 180) % 360) * 2
        """
        if len(msg) != 360:
            raise ValueError("sim_scan must contain exactly 360 ranges")

        real_scan = np.empty(360, dtype=np.float32)

        for i in range(360):
            real_scan[(360-i+180)%360] = msg[i]

       # second half‑degree

        return real_scan.tolist()
       
    
    def listener_callback(self,msg):
        movement_msg = Twist()

        msg.ranges = [100.0 if np.isinf(r) else r for r in msg.ranges]

        #msg.ranges = self.sim_to_real(msg.ranges)

        print(len(msg.ranges))

        # Get the maximum distances and their indices
        max_distances = self.max_distances(msg)

        viable_max_distance = self.check_viable_max(max_distances, msg)  

        print("Direction: ", viable_max_distance)
        print("Max distance: ", max_distances[0])


        movement_msg = self.follow_max(msg, movement_msg, viable_max_distance)
        # Check if the robot is too close to an obstacle
        


        self.last_laser_scan = msg
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
import rclpy
import time
from rclpy.node import Node
import statistics
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan



class DrivingNode(Node):

    def __init__(self):
        super().__init__('driving_node')
        self.turn_speed = .5
        self.movement_speed = 0.175
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan,'scan',self.listener_callback,10)
        

    def turnRight(self):
        movement_msg = Twist()
        movement_msg.linear.x = 0.0
        movement_msg.linear.y = 0.0
        movement_msg.linear.z = 0.0
        movement_msg.angular.x = 0.0
        movement_msg.angular.y = 0.0
        movement_msg.angular.z = -self.movement_speed
        self.publisher.publish(movement_msg)
        time.sleep(.675)
        movement_msg.angular.z = 0.0
        self.publisher.publish(movement_msg)

    def turnLeft(self):
        movement_msg = Twist()
        movement_msg.linear.x = 0.0
        movement_msg.linear.y = 0.0
        movement_msg.linear.z = 0.0
        movement_msg.angular.x = 0.0
        movement_msg.angular.y = 0.0
        movement_msg.angular.z = self.movement_speed
        self.publisher.publish(movement_msg)
        time.sleep(.675)
        movement_msg.angular.z = 0.0
        self.publisher.publish(movement_msg)

    def getMean(self,arr):
        sum = 0
        numVals = 0
        inf_count = 0
        for arr_val in arr:
            if (arr_val != float('inf')):
                sum += arr_val
                numVals +=1
            else:
                inf_count +=1
        if inf_count >=1:
            return 100.0
        else:
            return sum/numVals

    def listener_callback(self,msg):

        #l_forwardRange = msg.ranges[5:20]
        forwardRange = msg.ranges[355:360]+msg.ranges[0:5]
        #r_forwardRange = msg.ranges[335:355]

        #l_rightRange = msg.ranges[275:295]
        rightRange = msg.ranges[265:275]
        #r_rightRange = msg.ranges[245:265]

        #l_leftRange = msg.ranges[95:110]
        leftRange = msg.ranges[85:95]
        #r_leftRange = msg.ranges[70:85]

        fmedian = self.getMean(forwardRange)
        #l_fmedian = self.getMean(l_forwardRange)
        #r_fmedian = self.getMean(r_forwardRange)

        lmedian = self.getMean(leftRange)
        rmedian = self.getMean(rightRange)

        movement_msg = Twist()
        if ((fmedian > 0.4)):
            print("MOVING FORWARD")
            print("fmedian:" + str(fmedian))
            print("rmedian:" + str(rmedian))
            print("lmedian:" + str(lmedian))
            movement_msg.linear.x = self.movement_speed
            movement_msg.linear.y = 0.0
            movement_msg.linear.z = 0.0
            movement_msg.angular.x = 0.0
            movement_msg.angular.y = 0.0
            movement_msg.angular.z = 0.0
            self.publisher.publish(movement_msg)
            
        elif(rmedian > lmedian):
            print("TURNING RIGHT")
            print("fmedian:" + str(fmedian))
            print("rmedian:" + str(rmedian))
            print("lmedian:" + str(lmedian))
            self.turnRight()
        else:
            print("TURNING LEFT")
            print("fmedian:" + str(fmedian))
            print("rmedian:" + str(rmedian))
            print("lmedian:" + str(lmedian))
            self.turnLeft()

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
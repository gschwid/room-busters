import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import random
import math
import numpy as np
# The purpose of this node is to build off of dumb_vacuum to be slightly less dumb
# the proposed way of doing this, is to instead of looking straight forward, and viewing through a small
# field of view that leads to the robot "sticking" to the edges, what if we used the whole lidar, found the max 
# in a sliding window, say of 10 degrees, and then move in the center of that direction
# if there are multiple sliding windows that are optimal, we choose arbitrarily. 
# this adds a sense of randomness and might become completeness?!? total guess, but hopefully better

# 

class dumb_vacuum_v2(Node):
    def __init__(self):
        super().__init__('test')

        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.laser_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.odom_sub = self.create_subscription(
            Odometry, 
            'odom',
            self.odom_callback,
            10
        )
        self.compute_new_set = False
        self.angle_setpoint = 0.0

        # Store odometry so ET can find home. Effecively, init will be goal during pathing.
        # I think init and final should suffice as inputs to a path planning algorithm.
        self.initx = None
        self.inity = None
        self.finalx = None
        self.finaly = None

        # But when should ET go home? After the vacuum is done. We assume that when the program
        # runs, vacuuming is occuring up until a certain point.
        self.vacuuming = True

        # But when do we set self.vacuuming to false and use path planning?
        
        self.laser_sub  # prevent unused variable warning
        self.odom_message = Odometry()
    def get_yaw(self):
        if self.odom_message is None:
            return 0.0
        q = self.odom_message.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))
    def scan_callback(self, msg):
        # msg.ranges contains the distance readings
        # For TurtleBot3, index 0 is usually directly in front
        # Values are in meters
        # from the scan object we get angle min = 0 and max = 6.28
        # we get angle increment 0.017, this is about 1 degree 
        # so i will scan in the front 20 degrees that is -10 -> 0 
        # which is index 350-: and then 0->10 is  0:10
        if msg.ranges:
            # find max in sliding window 
            # instead of finding the max sliding window we just want to find a valid sliding window 
            # becuase otherwise we would avoid walls which isn't ideal
            move_message = TwistStamped()
            move_message.header.stamp = self.get_clock().now().to_msg()
            if not self.compute_new_set:


                sliding_window_len = 10
                min_dist = 0.5
                valid_starting_indices = []
                for i in range(len(msg.ranges)):
                    if i < len(msg.ranges):
                        arr = msg.ranges[i:i+10]
                        is_valid_range = (np.array(arr) > min_dist).all
                        if is_valid_range:
                            valid_starting_indices.append(i)
                    else:
                        first_arr = msg.ranges[i:]
                        second_arr = msg.ranges[0:sliding_window_len - len(first_arr)]
                        arr = first_arr = second_arr
                        is_valid_range = (arr > min_dist).all
                        if (is_valid_range):
                            valid_starting_indices.append(i)
                # once we have the valid starting indices 
                # we can select a random one 
                self.get_logger().info(f'valid_starting_indices: {valid_starting_indices}')
                rand_int = random.randint(0,len(valid_starting_indices))
                self.get_logger().info(f'rand_int?: {rand_int}')
                
                # then we want to turn to the value of that 
                angle_ind_to_turn_to = rand_int + 5
                angle = msg.angle_increment * angle_ind_to_turn_to + msg.range_min
                # then this must be relative to the current value of the robot, so we want to latch in a way 
                # so that when we are turning we only do this once.
                # then we need to PID the shiz out of this
                # we want to get the current angle of the robot from odometry 

                self.get_logger().info(f'dis working?: {self.get_yaw()}')

                # so now the desired place we want to turn to
                self.angle_setpoint = angle + self.get_yaw()
                self.compute_new_set = True
            else:
                pgain = 0.025
                error = self.angle_setpoint - self.get_yaw()
                self.get_logger().info(f'error: {error}')
                if error > 0.1: #some reasonable tolerance 
                    move_message.twist.angular.z = error * pgain
                else:
                    move_message.twist.linear.x = 1.0

            self.cmd_pub.publish(move_message)
    def odom_callback(self,msg): 
        self.odom_message = msg

        # odom_callback is where location subscription occurs. So this would be the place to set our variables.
        # if self.initx == None or self.inity = None :
        #    self.initx = self.odom_message.pose.pose.position.x
        #    self.inity = self.odom_message.pose.pose.position.y
        # 
        # self.finalx = self.odom_message.pose.pose.position.x
        # self.finaly = self.odom_message.pose.pose.position.y
 

    


def main(args=None):
    rclpy.init(args=args)
    test = dumb_vacuum_v2()
    rclpy.spin(test)
    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

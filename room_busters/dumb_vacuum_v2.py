# Dumb_vacuum_v2 
# fix issues from dumb_vacuum to avoid hugging walls by adding
# a probablistic element to the direction of travel

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import random
import math
import numpy as np
from enum import Enum

# small state manager
class state(Enum):
    FindNewDir = 1
    InFindDir = 2
    FollowingDir = 3


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

        self.battery = 100
        self.min_dist_lidar = 0.6
        self.angle_setpoint = 0.0
        self.state = state.FindNewDir
        self.recharge_val = 10

        self.laser_sub  # prevent unused variable warning
        self.odom_message = Odometry()

    # AI generated function for getting yaw from a quaternion
    def get_yaw(self):
        if self.odom_message is None:
            return 0.0
        q = self.odom_message.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))
    # most logic happens here in the scan callback
    def scan_callback(self, msg):
        # while I don't believe it is common practice to have 
        # logic in a sensor callback function
        # it is working, and I don't know what is common practice!

        # log battery state
        self.get_logger().info(f'battery: {self.battery}')

        # define battery threshold
        battery_above_threshold = self.battery > self.recharge_val

        if msg.ranges and battery_above_threshold:
            self.battery =self.battery-.1

            move_message = TwistStamped()
            move_message.header.stamp = self.get_clock().now().to_msg()

            # 40 degree field of view in front of robot
            distances_in_range = msg.ranges[340:] + msg.ranges[0:20]

            if(min(distances_in_range) < self.min_dist_lidar):
                self.get_logger().info(f'Need to find new direction...')
                if self.state == state.FollowingDir:
                    self.state = state.FindNewDir

            if self.state == state.FindNewDir:
                self.get_logger().info(f'Finding New Direction...')

                # ~ 10 degree sliding window
                sliding_window_len = 10
                valid_starting_indices = []
                
                # find the viable sliding windows
                for i in range(len(msg.ranges)):
                    # handling the edge effects of the array
                    if i+sliding_window_len <= len(msg.ranges):
                        arr = msg.ranges[i:i+sliding_window_len]
                        is_valid_range = (np.array(arr) > self.min_dist_lidar).all()
                        if is_valid_range:
                            valid_starting_indices.append(i)
                    else:
                        first_arr = msg.ranges[i:]
                        second_arr = msg.ranges[0:sliding_window_len - len(first_arr)]
                        arr = first_arr + second_arr
                        is_valid_range = (np.array(arr) > self.min_dist_lidar).all()
                        if (is_valid_range):
                            valid_starting_indices.append(i)
                # once we have the valid starting indices 
                # we can select a random one 
                self.get_logger().info(f'valid_starting_indices: {valid_starting_indices}')
                
                if not valid_starting_indices:
                    return

                # pick a random index 
                rand_list_idx = random.randint(0, len(valid_starting_indices)-1)
                
                # the scan index is the first of the sliding window
                chosen_scan_index = valid_starting_indices[rand_list_idx]
                
                # then we want to turn to the value of that 
                # add 5 to get the center of that sliding window
                angle_ind_to_turn_to = chosen_scan_index + 5

                # find angle
                angle_rad = msg.angle_increment * angle_ind_to_turn_to + msg.range_min
                
                # Convert to degrees before adding to yaw
                angle_deg = math.degrees(angle_rad)

                # so now the desired place we want to turn to
                self.angle_setpoint = angle_deg + self.get_yaw()
                self.state = state.InFindDir
            elif self.state == state.InFindDir:
                self.get_logger().info(f'Moving to new Direction...')
                pgain = 3.1
                error = self.angle_setpoint - self.get_yaw()
                
                # normalize error avoid wrap around 
                # ex. angle setpoint is -180, yaw is 170
                # error = -350 
                # normalized error = -350 + 180 = -170 
                # -170 % 360 - 190
                error = (error + 180) % 360 - 180
                error_rad = math.radians(error)
                self.get_logger().info(f'error: {error}')
                
                if abs(error) > 1.0: # 1 degree tolerance
                    move_message.twist.angular.z = error_rad * pgain
                else:
                    self.state = state.FollowingDir
            elif self.state == state.FollowingDir:
                self.get_logger().info(f'Following Chosen Direction...')
                move_message.twist.linear.x = 1.0

            self.cmd_pub.publish(move_message)
        else:
            move_message = TwistStamped()
            move_message.header.stamp = self.get_clock().now().to_msg()
            self.get_logger().info(f'Buster out of battery')
            self.cmd_pub.publish(move_message)


    def odom_callback(self,msg): 
        self.odom_message = msg


def main(args=None):
    rclpy.init(args=args)
    test = dumb_vacuum_v2()
    rclpy.spin(test)
    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

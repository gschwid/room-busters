import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan

# The purpose of this node is to build off of dumb_vacuum to be slightly less dumb
# the proposed way of doing this, is to instead of looking straight forward, and viewing through a small
# field of view that leads to the robot "sticking" to the edges, what if we used the whole lidar, found the max 
# in a sliding window, say of 10 degrees, and then move in the center of that direction
# if there are multiple sliding windows that are optimal, we choose arbitrarily. 
# this adds a sense of randomness and might become completeness?!? total guess, but hopefully better

class dumb_vacuum_v2(Node):
    def __init__(self):
        super().__init__('test')

        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.laser_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)


        self.laser_sub  # prevent unused variable warning

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
            sliding_window_len = 10
            self.distances_in_range = msg.ranges[340:] + msg.ranges[0:20]
            self.get_logger().info(f'Distance ahead: {self.distances_in_range}')
            move_message = TwistStamped()
            move_message.header.stamp = self.get_clock().now().to_msg()

            if(min(self.distances_in_range) > 0.7):
                self.get_logger().info(f'dis working?: {min(self.distances_in_range)}')

                # keep moving forward
                move_message.twist.linear.x = 5
                move_message.twist.angular.z = 0.0
            else:
                # rotate i guess until there is nothing in front of us?
                # what we could do, is rotate 360 in a "scanning pass" using the imu
                # then we could pick a random direction in the free space
                # other potentials are visualizing all of the places that we have been
                
                move_message.twist.linear.x = 0.0
                move_message.twist.angular.z = 1
            self.cmd_pub.publish(move_message)
            

def main(args=None):
    rclpy.init(args=args)
    test = dumb_vacuum()
    rclpy.spin(test)
    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

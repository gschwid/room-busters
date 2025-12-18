#Author: Henry Mahnke

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan


# need to publish to cmd_vel 
# want to get values from the lidar
class dumb_vacuum(Node):
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
        if msg.ranges:
            # define distances in field of view
            self.distances_in_range = msg.ranges[340:] + msg.ranges[0:20]
            # self.get_logger().info(f'Distance ahead: {self.distances_in_range}')
            move_message = TwistStamped()
            move_message.header.stamp = self.get_clock().now().to_msg()

            # check a minimum distance to see if collision
            if(min(self.distances_in_range) > 0.7):
                # no collision
                self.get_logger().info(f'dis working?: {min(self.distances_in_range)}')

                # move forward
                move_message.twist.linear.x = 5.0
                move_message.twist.angular.z = 0.0
            else:
                # yes collision, turn until no collision
                move_message.twist.linear.x = 0.0
                move_message.twist.angular.z = 1.0
            self.cmd_pub.publish(move_message)
            

def main(args=None):
    rclpy.init(args=args)
    test = dumb_vacuum()
    rclpy.spin(test)
    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

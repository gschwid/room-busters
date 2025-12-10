# # this node is to build off the dumb vacuum using grant's map 
# # I was originally going to use a package to do the coverage path planning 
# # but where is the fun in that?!?!
# # for this my idea is to try to sample in a rectangular fashion from the grid 
# # that i get from grant. Then i will path plan between those points. just point to point traversal 
# # with nav2. But you will run into obstacles won't you!?!
# # yes, in the case that I approach an obstacle i will use the "dumb" algorithm to just turn and move 
# # straight to try to get around it, before i go back to tracking that position. 
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import TwistStamped
# from sensor_msgs.msg import LaserScan
# from nav_msgs.msg import OccupancyGrid
# # need to publish to cmd_vel 
# # want to get values from the lidar
# class coverage_vacuum(Node):
#     def __init__(self):
#         super().__init__('test')

#         self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
#         self.laser_sub = self.create_subscription(
#             LaserScan,
#             'scan',
#             self.scan_callback,
#             10)

#         self.laser_sub  # prevent unused variable warning
#         self.map_sub = self.create_subscription(
#             OccupancyGrid, 
#             'occupancy',
#             self.map_callback,
#             10
#         )

#     def map_callback(self, msg):
#         # the occupancy grid naturally gives you the bottom left most coordinate
#         # all we really want to do is traverse every open cell, and we want the occupancy 
#         # grid to be such that the extremities of our robot will never clip any of the obstacles 
#         # so we need some type of buffer.
#         grid_x = index % width 
#         grid_y = index // width 
#         if msg.ranges:

#             self.distances_in_range = msg.ranges[340:] + msg.ranges[0:20]
#             self.get_logger().info(f'Distance ahead: {self.distances_in_range}')
#             move_message = TwistStamped()
#             move_message.header.stamp = self.get_clock().now().to_msg()

#             if(min(self.distances_in_range) > 0.7):
#                 self.get_logger().info(f'dis working?: {min(self.distances_in_range)}')

#                 # keep moving forward
#                 move_message.twist.linear.x = 1
#                 move_message.twist.angular.z = 0.0
#             else:
#                 # rotate i guess until there is nothing in front of us?
#                 # what we could do, is rotate 360 in a "scanning pass" using the imu
#                 # then we could pick a random direction in the free space
#                 # other potentials are visualizing all of the places that we have been
                
#                 move_message.twist.linear.x = 0.0
#                 move_message.twist.angular.z = 0.65
#             self.cmd_pub.publish(move_message)
            

# def main(args=None):
#     rclpy.init(args=args)
#     test = dumb_vacuum()
#     rclpy.spin(test)
#     test.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

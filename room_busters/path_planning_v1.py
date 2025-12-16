import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid

import random
import math
import numpy as np

# TODO: Finish porting over movement logic. Then implement algorithm.

class path_planning_v1(Node):
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

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
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
        # In dumb_vacuum_v2, 
        
        self.odom_message = Odometry()

        # The following way to track time heavily inspired by official jazzy documentation:
        # https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
        timer_period = 0.5
        self.i = 0
        self.timer = self.create_timer(timer_period, self.path_planning_v1)
        
    
    def path_planning_v1(self):

        # If we are still vacuuming, we follow a similar line of logic as vacuum().
        # I am deliberately trying to turn it into a trivial environment here though.
        if self.vacuuming :
            self.i += 1

            # TODO: Finish trivial movement logic
            move_message = TwistStamped()
            
            move_message.twist.linear.x = 1.0

            # Stop after 100 timesteps.
            if self.i >= 100 :
                self.vacuuming = False



        # Path back to origin after 100 timesteps.
        else:
            
            # We will probably use SLD for any heuristic i.e. astar. We'll see.
            sld = math.sqrt(((self.finalx - self.initx) ** 2) + ((self.finaly - self.inity) ** 2))

            # TODO:
            # result = dijkstras(self.initx, self.inity, self.finalx, self.finalx, map )
            
                
            
    
    def map_callback(self, msg):
        pass

    
    def odom_callback(self,msg): 
        self.odom_message = msg

        
        if self.initx == None or self.inity == None :
            self.initx = self.odom_message.pose.pose.position.x
            self.inity = self.odom_message.pose.pose.position.y
         
        self.finalx = self.odom_message.pose.pose.position.x
        self.finaly = self.odom_message.pose.pose.position.y
 

    

def main(args=None):
    rclpy.init(args=args)
    test = path_planning_v1()
    rclpy.spin(test)
    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

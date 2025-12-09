import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# need to publish to cmd_vel 
# want to get values from the lidar
class dumb_vacuum(Node):
    def __init__(self):
        super().__init__('test')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.subscription  # prevent unused variable warning
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('test node has been started.')

    def scan_callback(self, msg):
        # msg.ranges contains the distance readings
        # For TurtleBot3, index 0 is usually directly in front
        # Values are in meters
        if msg.ranges:
            distance_ahead = msg.ranges[0]
            self.get_logger().info(f'Distance ahead: {distance_ahead}')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = 0.1
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    test = Test()
    rclpy.spin(test)
    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

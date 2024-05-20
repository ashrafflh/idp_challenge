import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import random

class Jammer(Node):

    def __init__(self):
        super().__init__('jammer') # Initialize the node with its name
        self.get_logger().info('GPS jammer node has been started...') # print a log message
        # Create a subscriber with its callback to the topic where the position data from GPS is being written
        self.gps_pos_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_pos_callback,
            10)
        # Create a publisher that will publish the jammed positional GPS data to a topic
        self.jammed_gps_pub = self.create_publisher(NavSatFix, '/gps/jammed', 10)

    def gps_pos_callback(self, msg):
        # Simulate jammed GPS data
        jammed_msg = NavSatFix()
        jammed_msg.header = msg.header
        jammed_msg.status = msg.status
        jammed_msg.latitude = msg.latitude + self.random_error()
        jammed_msg.longitude = msg.longitude + self.random_error()
        jammed_msg.altitude = msg.altitude + self.random_error()
        jammed_msg.position_covariance = msg.position_covariance
        jammed_msg.position_covariance_type = msg.position_covariance_type

        self.jammed_gps_pub.publish(jammed_msg)
        self.get_logger().info(f'Published jammed GPS data: {jammed_msg.latitude}, {jammed_msg.longitude}, {jammed_msg.altitude}')

    def random_error(self):
        # Introduce a random error between -100.0 and 100.0 cms
        return random.uniform(-100.0, 100.0)


def main(args=None):
    rclpy.init(args=args)
    gps_error_injector = Jammer()
    rclpy.spin(gps_error_injector)
    gps_error_injector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
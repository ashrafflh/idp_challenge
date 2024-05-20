import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

class JumpDetection(Node): # create a class for the jump_detection node

    def __init__(self):
        super().__init__('jump_detection_node') # Initialize the node with its name
        self.get_logger().info('Jump detection node has been started...') # print a log message

        self.declare_parameter('jump_distance', 50.0) # declare a node parameter used to define the distance for a significant jump
        
        self.jump_distance = self.get_parameter('jump_distance').get_parameter_value().double_value

        # Create subscriber with its callback to the topic where the position data from GPS is being written
        self.gps_pos_sub = self.create_subscription(
            NavSatFix,
            '/gps/jammed',
            self.gps_pos_callback,
            10
        )

        # Publisher for jump detection topic
        self.jump_detection_pub = self.create_publisher(String, '/jump_detection', 10)

        
        # Create variable to hold the previous position of the robot
        self.previous_gps_pos = None

    
    def gps_pos_callback(self, pos: NavSatFix):
        self.get_logger().info('AAA...')
        current_pos = self.get_position_from_gps_msg(pos)
        if self.previous_gps_pos is not None:
            self.detect_jumps_gps(current_pos)
        self.previous_gps_pos = current_pos

    # a function to read position data from GPS
    def get_position_from_gps_msg(self, pos: NavSatFix):
        # currently using latitude, longitude and altitude only.
        return np.array([pos.latitude, pos.longitude, pos.altitude
                         ])

    # a function to detect the jumps in the position
    def detect_jumps_gps(self, current_pos):
        distance = np.linalg.norm(current_pos - self.previous_gps_pos)
        if distance > self.jump_distance:
            # Write to log
            self.get_logger().warn(f'Jump in position detected! Distance: {distance}')
            # Write to topic
            detection_msg = String()
            detection_msg.data = f'Jump in position detected! Distance: {distance}'
            self.jump_detection_pub.publish(detection_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JumpDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
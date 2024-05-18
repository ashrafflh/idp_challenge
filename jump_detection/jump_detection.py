import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

class JumpDetection(Node): # create a class for the jump_detection node

    def __init__(self):
        super().__init__('jump_detection_node') # Initialize the node with its name
        self.get_logger().info('Jump detection node has been started...') # print a log message

        self.declare_parameter('jump_distance', 50.0) # declare a node parameter used to define the distance for a significant jump
        
        self.jump_distance = self.get_parameter('jump_distance').get_parameter_value().double_value
        """
        # Create subscribers with their respective callbacks to the topics where the local and global position data from the EKFs is being stored
        self.pos_data_local_sub = self.create_subscription(
            Odometry,
            '/odometry/local',
            self.pos_data_local_callback,
            10
        )
        
        self.pos_data_global_sub = self.create_subscription(
            Odometry,
            '/odometry/global',
            self.pos_data_global_callback,
            10
        )
        """
        self.gps_pos_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_pos_callback,
            10
        )
        # Publisher for jump detection
        self.jump_detection_pub = self.create_publisher(String, '/jump_detection', 10)

        
        # Create variables to hold the local and global position of the robot
        self.local_pos = None
        self.global_pos = None
        self.gps_pos = None
        self.previous_gps_pos = None

    def pos_data_local_callback(self, pos: Odometry):
        self.local_pos = self.get_position_from_pos(pos)
        self.detect_jumps()

    def pos_data_global_callback(self, pos: Odometry):
        self.global_pos = self.get_position_from_pos(pos)
        self.detect_jumps()
    
    def gps_pos_callback(self, pos: NavSatFix):
        current_pos = self.get_position_from_gps_msg(pos)
        if self.previous_gps_pos is not None:
            self.detect_jumps_gps(current_pos)
        self.previous_gps_pos = current_pos

    # a function to read position data from Odemetry and give them as an array back
    def get_position_from_pos(self, pos: Odometry):
        return np.array([pos.pose.pose.position.x, pos.pose.pose.position.y, pos.pose.pose.position.z])

    # a function to read position data from GPS
    def get_position_from_gps_msg(self, pos: NavSatFix):
        # For simplicity, using latitude and longitude only. Convert to a local Cartesian frame if necessary.
        return np.array([pos.latitude, pos.longitude])


    # a function to detect the jumps in the positions
    def detect_jumps(self):
        if self.local_pos is not None and self.global_pos is not None:
            distance = np.linalg.norm(self.local_pos - self.global_pos)
            if distance > self.jump_distance:
                # Write to log
                self.get_logger().warn(f'Jump in position detected! Distance: {distance}')
                # Write to topic
                detection_msg = String()
                detection_msg.data = f'Jump in position detected! Distance: {distance}'
                self.jump_detection_pub.publish(detection_msg)

    # a function to detect the jumps in the positions
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
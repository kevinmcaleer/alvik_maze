#!/usr/bin/env python3
"""
Robot State Publisher for Alvik
Publishes the robot URDF description and static transforms
"""
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os


class AlvikRobotPublisher(Node):
    def __init__(self):
        super().__init__('alvik_robot_publisher')

        # Read URDF file
        package_dir = get_package_share_directory('alvik_navigation')
        urdf_file = os.path.join(package_dir, 'urdf', 'alvik.urdf')

        try:
            with open(urdf_file, 'r') as f:
                robot_description = f.read()
        except FileNotFoundError:
            self.get_logger().error(f'URDF file not found: {urdf_file}')
            return

        # Publish robot_description parameter
        self.declare_parameter('robot_description', robot_description)

        self.get_logger().info('Robot description published!')
        self.get_logger().info(f'Loaded URDF from: {urdf_file}')


def main(args=None):
    rclpy.init(args=args)
    node = AlvikRobotPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

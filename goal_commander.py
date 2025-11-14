#!/usr/bin/env python3
"""
Goal Commander Node - Receives goal poses from rviz2 (2D Goal Pose tool)
and sends movement commands to Alvik via MQTT
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import paho.mqtt.client as mqtt
import math
import json

class GoalCommander(Node):
    def __init__(self):
        super().__init__('goal_commander')
        
        # Declare parameters
        self.declare_parameter('mqtt_host', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        
        # Get parameters
        mqtt_host = self.get_parameter('mqtt_host').value
        mqtt_port = self.get_parameter('mqtt_port').value
        
        # Current robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Goal state
        self.goal_x = None
        self.goal_y = None
        self.goal_theta = None
        self.goal_active = False
        
        # Subscribe to goal poses from rviz2
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # Subscribe to odometry (robot position updates)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Publish velocity commands (for visualization)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # MQTT client for sending commands to Alvik
        self.mqtt_client = mqtt.Client(client_id='ros2_goal_commander')
        self.mqtt_client.on_connect = self.on_mqtt_connect
        
        try:
            self.mqtt_client.connect(mqtt_host, mqtt_port, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info(f'Goal Commander connected to MQTT at {mqtt_host}:{mqtt_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MQTT: {e}')
        
        # Control loop timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Goal Commander started - Click "2D Goal Pose" in rviz2 to navigate!')
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback when MQTT connects"""
        if rc == 0:
            self.get_logger().info('MQTT connected for goal commands')
        else:
            self.get_logger().error(f'MQTT connection failed: {rc}')
    
    def goal_callback(self, msg):
        """Receive goal pose from rviz2"""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        
        # Extract yaw from quaternion
        quat = msg.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        self.goal_theta = math.atan2(siny_cosp, cosy_cosp)
        
        self.goal_active = True
        
        self.get_logger().info(
            f'New goal received: ({self.goal_x:.2f}, {self.goal_y:.2f}, {math.degrees(self.goal_theta):.1f}°)'
        )
    
    def odom_callback(self, msg):
        """Update current robot position from odometry"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract yaw
        quat = msg.pose.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)
    
    def control_loop(self):
        """Simple proportional controller to move toward goal"""
        if not self.goal_active or self.goal_x is None:
            return
        
        # Calculate distance to goal
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Calculate angle to goal
        angle_to_goal = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_goal - self.current_theta)
        
        # Thresholds
        distance_threshold = 0.05  # 5cm
        angle_threshold = 0.1  # ~6 degrees
        
        if distance < distance_threshold:
            # Close enough - now align to goal orientation
            final_angle_error = self.normalize_angle(self.goal_theta - self.current_theta)
            
            if abs(final_angle_error) < angle_threshold:
                # Goal reached!
                self.stop_robot()
                self.goal_active = False
                self.get_logger().info('Goal reached!')
                return
            else:
                # Rotate to final orientation
                self.send_rotate_command(final_angle_error)
        else:
            # Still need to move toward goal
            if abs(angle_error) > angle_threshold:
                # First align toward goal
                self.send_rotate_command(angle_error)
            else:
                # Move forward
                self.send_move_command(distance)
    
    def send_move_command(self, distance):
        """Send move forward command to Alvik"""
        # Convert meters to cm
        distance_cm = distance * 100.0
        
        # Send via MQTT
        command = {
            'type': 'move',
            'distance': round(distance_cm, 1)
        }
        self.mqtt_client.publish('alvik/command', json.dumps(command))
        
        # Publish cmd_vel for visualization
        twist = Twist()
        twist.linear.x = 0.1  # 10 cm/s
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().debug(f'Moving forward {distance_cm:.1f}cm')
    
    def send_rotate_command(self, angle_rad):
        """Send rotate command to Alvik"""
        # Convert radians to degrees
        angle_deg = math.degrees(angle_rad)
        
        # Send via MQTT
        command = {
            'type': 'rotate',
            'angle': round(angle_deg, 1)
        }
        self.mqtt_client.publish('alvik/command', json.dumps(command))
        
        # Publish cmd_vel for visualization
        twist = Twist()
        twist.angular.z = 0.5 if angle_rad > 0 else -0.5
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().debug(f'Rotating {angle_deg:.1f}°')
    
    def stop_robot(self):
        """Stop the robot"""
        command = {'type': 'stop'}
        self.mqtt_client.publish('alvik/command', json.dumps(command))
        
        twist = Twist()  # All zeros
        self.cmd_vel_pub.publish(twist)
    
    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = GoalCommander()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt_client.loop_stop()
        node.mqtt_client.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

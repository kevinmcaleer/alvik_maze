#!/usr/bin/env python3
"""
Maze Explorer Node for Alvik Robot
Uses wall following and reactive navigation to explore and solve mazes
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import paho.mqtt.client as mqtt
import json
import math
import time
from enum import Enum


class ExplorationState(Enum):
    FOLLOW_WALL = 1
    AT_INTERSECTION = 2
    AT_DEAD_END = 3
    SCANNING_360 = 4
    TURNING = 5


class MazeExplorer(Node):
    def __init__(self):
        super().__init__('maze_explorer')

        # Parameters
        self.declare_parameter('mqtt_host', '192.168.1.152')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('wall_distance', 0.15)  # Target distance from wall (15cm)
        self.declare_parameter('forward_speed', 5.0)   # cm/s
        self.declare_parameter('turn_speed', 30.0)     # deg/s

        mqtt_host = self.get_parameter('mqtt_host').value
        mqtt_port = self.get_parameter('mqtt_port').value
        self.wall_distance = self.get_parameter('wall_distance').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value

        # State
        self.state = ExplorationState.FOLLOW_WALL
        self.last_scan = None
        self.last_odom = None
        self.exploration_active = False

        # Wall following parameters
        self.min_front_distance = 0.25  # Stop if wall closer than 25cm ahead
        self.intersection_threshold = 0.40  # Gap wider than 40cm = intersection

        # Subscribe to ROS topics
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Connect to MQTT for robot control
        self.mqtt_client = mqtt.Client(client_id="maze_explorer")
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.connect(mqtt_host, mqtt_port, 60)
        self.mqtt_client.loop_start()

        # Timer for exploration logic
        self.timer = self.create_timer(0.1, self.exploration_loop)  # 10 Hz

        self.get_logger().info('Maze Explorer Started!')
        self.get_logger().info('Publish "start" to alvik/explore to begin exploration')

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        self.get_logger().info(f'Connected to MQTT broker: {rc}')
        # Subscribe to exploration control topic
        self.mqtt_client.subscribe("alvik/explore")

    def on_mqtt_message(self, client, userdata, msg):
        """Handle MQTT messages"""
        try:
            if msg.topic == "alvik/explore":
                command = msg.payload.decode()
                if command == "start":
                    self.exploration_active = True
                    self.state = ExplorationState.FOLLOW_WALL
                    self.get_logger().info('Starting maze exploration!')
                elif command == "stop":
                    self.exploration_active = False
                    self.send_stop_command()
                    self.get_logger().info('Stopping maze exploration')
        except Exception as e:
            self.get_logger().error(f'Error handling MQTT message: {e}')

    def scan_callback(self, msg):
        """Receive laser scan data"""
        self.last_scan = msg

    def odom_callback(self, msg):
        """Receive odometry data"""
        self.last_odom = msg

    def get_scan_regions(self):
        """
        Divide scan into regions: left, front, right
        Returns average distances in each region
        """
        if self.last_scan is None:
            return None, None, None

        ranges = self.last_scan.ranges
        if len(ranges) == 0:
            return None, None, None

        # Assuming front-facing scan centered at 0°
        # Split into 3 regions based on angle
        angle_min = self.last_scan.angle_min
        angle_max = self.last_scan.angle_max
        angle_inc = self.last_scan.angle_increment

        left_ranges = []
        front_ranges = []
        right_ranges = []

        for i, r in enumerate(ranges):
            if r <= 0 or r > self.last_scan.range_max:
                continue

            angle = angle_min + i * angle_inc
            # Normalize to [-pi, pi]
            angle = math.atan2(math.sin(angle), math.cos(angle))

            # Split: left > 15°, front ±15°, right < -15°
            if angle > math.radians(15):
                left_ranges.append(r)
            elif angle < math.radians(-15):
                right_ranges.append(r)
            else:
                front_ranges.append(r)

        # Calculate averages
        left_dist = sum(left_ranges) / len(left_ranges) if left_ranges else float('inf')
        front_dist = sum(front_ranges) / len(front_ranges) if front_ranges else float('inf')
        right_dist = sum(right_ranges) / len(right_ranges) if right_ranges else float('inf')

        return left_dist, front_dist, right_dist

    def exploration_loop(self):
        """Main exploration logic - runs at 10 Hz"""
        if not self.exploration_active:
            return

        if self.last_scan is None:
            return

        # Get distances in each region
        left_dist, front_dist, right_dist = self.get_scan_regions()

        if left_dist is None:
            return

        # State machine for exploration
        if self.state == ExplorationState.FOLLOW_WALL:
            self.follow_right_wall(left_dist, front_dist, right_dist)

        elif self.state == ExplorationState.AT_INTERSECTION:
            self.handle_intersection(left_dist, front_dist, right_dist)

        elif self.state == ExplorationState.AT_DEAD_END:
            self.handle_dead_end()

    def follow_right_wall(self, left_dist, front_dist, right_dist):
        """
        Right-hand wall following algorithm
        """
        # Check for obstacles ahead
        if front_dist < self.min_front_distance:
            # Wall ahead - check for intersection or dead end
            if left_dist > self.intersection_threshold:
                # Can turn left - intersection
                self.get_logger().info('Intersection detected (wall ahead, left open)')
                self.state = ExplorationState.AT_INTERSECTION
                self.send_stop_command()
                return
            elif right_dist > self.intersection_threshold:
                # Can turn right - intersection
                self.get_logger().info('Intersection detected (wall ahead, right open)')
                self.send_turn_command(90)  # Turn right immediately (right-hand rule)
                return
            else:
                # Dead end
                self.get_logger().info('Dead end detected')
                self.state = ExplorationState.AT_DEAD_END
                self.send_stop_command()
                return

        # Check for right turn opportunity (right-hand rule priority)
        if right_dist > self.intersection_threshold:
            self.get_logger().info('Right turn available - taking it')
            self.send_turn_command(90)
            return

        # Normal wall following
        # Try to maintain target distance from right wall
        error = right_dist - self.wall_distance

        if abs(error) < 0.05:  # Within 5cm tolerance
            # Move straight
            self.send_move_command(self.forward_speed)
        elif error > 0:
            # Too far from wall - turn slightly right
            self.send_move_command(self.forward_speed * 0.7)
            self.send_turn_command(15)
        else:
            # Too close to wall - turn slightly left
            self.send_move_command(self.forward_speed * 0.7)
            self.send_turn_command(-15)

    def handle_intersection(self, left_dist, front_dist, right_dist):
        """
        Handle intersection - trigger 360° scan and choose direction
        """
        self.get_logger().info('Scanning intersection...')

        # Trigger 360° scan
        self.mqtt_client.publish('alvik/command', json.dumps({'type': 'scan'}))

        # For now, prefer right (right-hand rule), then straight, then left
        if right_dist > self.intersection_threshold:
            self.get_logger().info('Turning right at intersection')
            self.send_turn_command(90)
        elif front_dist > self.min_front_distance:
            self.get_logger().info('Going straight at intersection')
            self.send_move_command(20)  # Move 20cm forward
        elif left_dist > self.intersection_threshold:
            self.get_logger().info('Turning left at intersection')
            self.send_turn_command(-90)
        else:
            # Shouldn't happen, but turn around if stuck
            self.get_logger().warn('No clear path - turning around')
            self.send_turn_command(180)

        # Return to wall following
        self.state = ExplorationState.FOLLOW_WALL

    def handle_dead_end(self):
        """
        Handle dead end - turn around 180°
        """
        self.get_logger().info('Turning around (180°)')
        self.send_turn_command(180)

        # Trigger 360° scan at dead end
        self.mqtt_client.publish('alvik/command', json.dumps({'type': 'scan'}))

        # Return to wall following
        time.sleep(2)  # Wait for turn to complete
        self.state = ExplorationState.FOLLOW_WALL

    def send_move_command(self, distance_cm):
        """Send move command to robot via MQTT"""
        cmd = {
            'type': 'move',
            'distance': distance_cm
        }
        self.mqtt_client.publish('alvik/command', json.dumps(cmd))

    def send_turn_command(self, angle_deg):
        """Send rotation command to robot via MQTT"""
        cmd = {
            'type': 'rotate',
            'angle': angle_deg
        }
        self.mqtt_client.publish('alvik/command', json.dumps(cmd))

    def send_stop_command(self):
        """Send stop command to robot via MQTT"""
        cmd = {'type': 'stop'}
        self.mqtt_client.publish('alvik/command', json.dumps(cmd))

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MazeExplorer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

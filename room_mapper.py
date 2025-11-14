#!/usr/bin/env python3
"""
Room Mapper Node - Converts LaserScan data to an occupancy grid map
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
import numpy as np
import math

class RoomMapper(Node):
    def __init__(self):
        super().__init__('room_mapper')
        
        # Declare parameters
        self.declare_parameter('map_size_meters', 5.0)
        self.declare_parameter('map_resolution', 0.02)  # 2cm per cell
        self.declare_parameter('occupied_threshold', 50)
        
        # Get parameters
        map_size = self.get_parameter('map_size_meters').value
        self.resolution = self.get_parameter('map_resolution').value
        self.occupied_threshold = self.get_parameter('occupied_threshold').value
        
        # Calculate grid size
        self.grid_size = int(map_size / self.resolution)
        
        # Initialize occupancy grid (0 = free, 100 = occupied, -1 = unknown)
        self.grid = np.full((self.grid_size, self.grid_size), -1, dtype=np.int8)
        
        # Robot starts at center
        self.robot_x = self.grid_size // 2
        self.robot_y = self.grid_size // 2
        
        # Subscribe to laser scans
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        
        # Publish occupancy grid
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 10)
        
        # Publish map periodically
        self.timer = self.create_timer(1.0, self.publish_map)
        
        self.get_logger().info(f'Room Mapper initialized: {self.grid_size}x{self.grid_size} grid at {self.resolution}m resolution')
    
    def scan_callback(self, scan_msg):
        """Process incoming laser scan and update occupancy grid"""
        
        # Clear cells along scan rays (mark as free)
        for i, distance in enumerate(scan_msg.ranges):
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            
            # Skip invalid readings
            if math.isinf(distance) or math.isnan(distance):
                continue
            
            if distance < scan_msg.range_min or distance > scan_msg.range_max:
                continue
            
            # Calculate obstacle position in grid coordinates
            obs_x = self.robot_x + int((distance * math.cos(angle)) / self.resolution)
            obs_y = self.robot_y + int((distance * math.sin(angle)) / self.resolution)
            
            # Mark cells along the ray as free using Bresenham's line algorithm
            free_cells = self.bresenham_line(self.robot_x, self.robot_y, obs_x, obs_y)
            
            for (fx, fy) in free_cells[:-1]:  # All except last cell
                if 0 <= fx < self.grid_size and 0 <= fy < self.grid_size:
                    if self.grid[fy, fx] == -1:  # Only mark unknown cells
                        self.grid[fy, fx] = 0  # Free space
            
            # Mark obstacle cell as occupied
            if 0 <= obs_x < self.grid_size and 0 <= obs_y < self.grid_size:
                self.grid[obs_y, obs_x] = 100  # Occupied
        
        self.get_logger().debug(f'Processed scan with {len(scan_msg.ranges)} rays')
    
    def bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's line algorithm to get all cells along a ray"""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            cells.append((x, y))
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return cells
    
    def publish_map(self):
        """Publish the current occupancy grid"""
        map_msg = OccupancyGrid()
        
        # Header
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        
        # Map metadata
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.grid_size
        map_msg.info.height = self.grid_size
        
        # Origin is at bottom-left of map, with robot at center
        map_msg.info.origin.position.x = -(self.grid_size * self.resolution) / 2.0
        map_msg.info.origin.position.y = -(self.grid_size * self.resolution) / 2.0
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        
        # Flatten grid data (row-major order)
        map_msg.data = self.grid.flatten().tolist()
        
        self.map_pub.publish(map_msg)
        
        # Log statistics
        occupied_cells = np.sum(self.grid == 100)
        free_cells = np.sum(self.grid == 0)
        unknown_cells = np.sum(self.grid == -1)
        
        self.get_logger().debug(
            f'Map: {occupied_cells} occupied, {free_cells} free, {unknown_cells} unknown cells'
        )

def main(args=None):
    rclpy.init(args=args)
    node = RoomMapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

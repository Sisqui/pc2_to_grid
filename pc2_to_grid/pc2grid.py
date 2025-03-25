#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs_py.point_cloud2 import read_points_numpy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from geometry_msgs.msg import Pose
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class PointCloudToOccupancyGrid(Node):

    def __init__(self):
        super().__init__('pointcloud_to_occupancy_grid_node')

        # Declare parameters with default values
        self.declare_parameter("pc2_topic", "/cloud_transformed")
        self.declare_parameter("odom_topic", "/zed/zed_node/odom")
        self.declare_parameter("occupancy_grid_topic", "/occupancy_grid_scan")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("grid_resolution", 0.1)
        self.declare_parameter("grid_width", 1000)
        self.declare_parameter("grid_height", 1000)
        self.declare_parameter("max_range", 10.0)

        # Get parameter values
        self.pc2_topic = self.get_parameter("pc2_topic").get_parameter_value().string_value
        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.occupancy_grid_topic = self.get_parameter("occupancy_grid_topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.grid_resolution = self.get_parameter("grid_resolution").get_parameter_value().double_value
        self.grid_width = self.get_parameter("grid_width").get_parameter_value().integer_value
        self.grid_height = self.get_parameter("grid_height").get_parameter_value().integer_value
        self.max_range = self.get_parameter("max_range").get_parameter_value().double_value

        # Initialize robot position in the occupancy grid
        self.robot_x = self.grid_width * self.grid_resolution / 2.0
        self.robot_y = self.grid_height * self.grid_resolution / 2.0

        self.occupancy_grid = np.full((self.grid_height, self.grid_width), -1, dtype=int)

        # Publisher for the OccupancyGrid message
        self.occupancy_grid_publisher = self.create_publisher(OccupancyGrid, self.occupancy_grid_topic, 10)

        # Define a Best Effort QoS profile
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )


        # Subscriber for the PC2 and Odometry topics
        self.scan_subscriber = self.create_subscription(
            PointCloud2, self.pc2_topic, self.pc2_callback, best_effort_qos
        )
        self.odom_subscriber = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, best_effort_qos
        )

        # Set up the metadata for the OccupancyGrid message
        self.map_metadata = MapMetaData()
        self.map_metadata.resolution = self.grid_resolution
        self.map_metadata.width = self.grid_width
        self.map_metadata.height = self.grid_height
        self.map_metadata.origin = Pose()
        self.map_metadata.origin.position.x = -self.grid_width * self.grid_resolution / 2.0
        self.map_metadata.origin.position.y = -self.grid_height * self.grid_resolution / 2.0
        self.map_metadata.origin.orientation.w = 1.0

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def pc2_callback(self, msg):
        robot_cell_x = int((self.robot_x - self.map_metadata.origin.position.x) / self.grid_resolution)
        robot_cell_y = int((self.robot_y - self.map_metadata.origin.position.y) / self.grid_resolution)

        points = read_points_numpy(msg)
        # print(len(points))
        # Iterate through each range in the PointCloud message
        for p in points:

            x = p[0]
            y = p[1]

            end_cell_x = int((x - self.map_metadata.origin.position.x) / self.grid_resolution)
            end_cell_y = int((y - self.map_metadata.origin.position.y) / self.grid_resolution)

            cells = self.bresenham(robot_cell_x, robot_cell_y, end_cell_x, end_cell_y)

            for (cx, cy) in cells[:-1]:  # Exclude the last cell which is the occupied endpoint
                if 0 <= cx < self.grid_width and 0 <= cy < self.grid_height:
                    # self.occupancy_grid[cy, cx] = 0  # Free space
                    self.occupancy_grid[cy, cx] -= 0.2
                    if self.occupancy_grid[cy, cx] < 0 :
                        self.occupancy_grid[cy, cx] = 0

            if 0 <= end_cell_x < self.grid_width and 0 <= end_cell_y < self.grid_height:
                # self.occupancy_grid[end_cell_y, end_cell_x] = 100  # Occupied space
                self.occupancy_grid[end_cell_y, end_cell_x] += 10
                if self.occupancy_grid[end_cell_y, end_cell_x]  > 100 :
                    self.occupancy_grid[end_cell_y, end_cell_x] = 100

        occupancy_grid_msg = OccupancyGrid()
        occupancy_grid_msg.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid_msg.header.frame_id = self.frame_id
        occupancy_grid_msg.info = self.map_metadata
        occupancy_grid_msg.data = self.occupancy_grid.flatten().tolist()

        # print("Publishing map...")
        self.occupancy_grid_publisher.publish(occupancy_grid_msg)

    def bresenham(self, x0, y0, x1, y1):
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return cells

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToOccupancyGrid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

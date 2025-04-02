import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

class PointCloudToImage(Node):
    def __init__(self):
        super().__init__('pointcloud_to_image')
        
        # Declare parameters with default values
        self.declare_parameter("pc2_topic", "/bonxai_point_cloud_centers")
        self.declare_parameter("map_name", "my_3dmap.png")
        self.declare_parameter("rotation_angle", 45.0)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)

        # Get parameter values
        self.pc2_topic = self.get_parameter("pc2_topic").get_parameter_value().string_value
        self.map_name = self.get_parameter("map_name").get_parameter_value().string_value
        self.rotation_angle = self.get_parameter("rotation_angle").value
        self.image_size = (
            self.get_parameter("image_width").value,
            self.get_parameter("image_height").value
        )
        
        self.subscription = self.create_subscription(PointCloud2, self.pc2_topic, self.pointcloud_callback, 10)

    def pointcloud_callback(self, msg):
        try:
            has_rgb = any(f.name == 'rgb' for f in msg.fields)
            field_names = ["x", "y", "z"] + (["rgb"] if has_rgb else [])
            
            # Read points
            points = point_cloud2.read_points_list(
                msg, 
                field_names=field_names,
                skip_nans=True
            )
            
            if not points:
                self.get_logger().warn("Empty point cloud received!")
                return

            # Convert to numpy arrays
            x = np.array([p[0] for p in points])
            y = np.array([p[1] for p in points])
            z = np.array([p[2] for p in points])
            
            # Rotate points
            theta = np.radians(self.rotation_angle)
            rotation_matrix = np.array([
                [1, 0, 0],
                [0, np.cos(theta), -np.sin(theta)],
                [0, np.sin(theta), np.cos(theta)]
            ])
            rotated = np.dot(rotation_matrix, np.vstack([x, y, z]))

            # Project to 2D
            x_proj = rotated[0]
            y_proj = rotated[1]

            # Normalize coordinates
            x_min, x_max = np.min(x_proj), np.max(x_proj)
            y_min, y_max = np.min(y_proj), np.max(y_proj)
            
            if x_max == x_min or y_max == y_min:
                self.get_logger().warn("All points are identical!")
                return

            x_img = ((x_proj - x_min) * (self.image_size[0]-1) / (x_max - x_min)).astype(int)
            y_img = ((y_proj - y_min) * (self.image_size[1]-1) / (y_max - y_min)).astype(int)

            # Create image
            image = np.zeros((self.image_size[1], self.image_size[0], 3), dtype=np.uint8)

            # Assign colors
            if has_rgb:
                rgb = np.array([p[3] for p in points])
                colors = np.array([self.unpack_rgb(r) for r in rgb])
            else:
                z_norm = (z - np.min(z)) / (np.max(z) - np.min(z))
                colors = cv2.applyColorMap((z_norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
                colors = colors.squeeze()  # Fix shape from (N,1,3) to (N,3)

            # Draw points
            valid = (x_img >= 0) & (x_img < self.image_size[0]) & \
                   (y_img >= 0) & (y_img < self.image_size[1])
            
            image[y_img[valid], x_img[valid]] = colors[valid]

            cv2.imwrite(self.map_name, image)
            self.get_logger().info("Image saved successfully!")

        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {str(e)}")

    def unpack_rgb(self, rgb_float):
        rgb_int = int(rgb_float * (2**24 - 1))
        return [
            rgb_int & 0xFF,          # Blue
            (rgb_int >> 8) & 0xFF,   # Green
            (rgb_int >> 16) & 0xFF   # Red
        ]

def main():
    rclpy.init()
    node = PointCloudToImage()
    rclpy.spin_once(node)

if __name__ == '__main__':
    main()
    
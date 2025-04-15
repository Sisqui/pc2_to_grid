# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import OccupancyGrid
# import numpy as np
# from PIL import Image

# class OccupancyGridSaver(Node):
#     def __init__(self):
#         super().__init__('occupancy_grid_saver')

#         # Declare parameters with default values
#         self.declare_parameter("map_topic", "/occupancy_grid_scan")
#         self.declare_parameter("map_name", "my_map.pgm")

#         # Get parameter values
#         self.map_topic = self.get_parameter("map_topic").get_parameter_value().string_value
#         self.map_name = self.get_parameter("map_name").get_parameter_value().string_value

#         self.map_subscriber = self.create_subscription(
#             OccupancyGrid,
#             self.map_topic,
#             self.listener_callback,
#             10
#         )
#         self.get_logger().info('Subscribed to /occupancy_grid_scan')

#     def listener_callback(self, msg: OccupancyGrid):
#         width = msg.info.width
#         height = msg.info.height
#         data = np.array(msg.data, dtype=np.int8).reshape(height, width)

#         # Convert data to PGM format (ROS uses -1 for unknown, convert to 205)
#         pgm_data = np.zeros_like(data, dtype=np.uint8)
#         pgm_data[data == -1] = 205  # Unknown
#         pgm_data[data == 0] = 254   # Free
#         pgm_data[data > 0] = 0      # Occupied

#         # Save as PGM using PIL
#         image = Image.fromarray(pgm_data)

#         # Resize the image (downsampling by 50%)
#         new_width = width // 4
#         new_height = height // 4
#         image_resized = image.resize((new_width, new_height))

#         image_resized.save(self.map_name)
#         self.get_logger().info('Saved occupancy grid')

#         # Shutdown node after saving
#         self.destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = OccupancyGridSaver()
#     rclpy.spin_once(node)

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from PIL import Image
import os

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        
        # Declare parameters with default values
        self.declare_parameter("map_topic", "/mapUAV")
        self.declare_parameter("map_name", "my_map.pgm")

        # Get parameter values
        self.map_topic = self.get_parameter("map_topic").get_parameter_value().string_value
        self.map_name = self.get_parameter("map_name").get_parameter_value().string_value

        self.map_subscriber = self.create_subscription(OccupancyGrid, self.map_topic, self.listener_callback, 10)

    def listener_callback(self, msg):
        width, height = msg.info.width, msg.info.height
        self.get_logger().info(f"Received map: width={width}, height={height}")

        try:
            grid_data = np.array(msg.data, dtype=np.int8).reshape((height, width))

            # Create grayscale image based on occupancy values
            image = np.zeros((height, width), dtype=np.uint8)
            image[grid_data == -1] = 128  # Unknown
            image[grid_data == 0] = 255   # Free
            image[grid_data == 100] = 0   # Occupied

            # Save using Pillow
            map_path = os.path.join(os.path.expanduser("~"), "my_map.pgm")
            img_pil = Image.fromarray(image)

            img_pil = img_pil.transpose(Image.FLIP_TOP_BOTTOM)

            # scale_factor = 4
            # new_size = (width * scale_factor, height * scale_factor)
            # img_resized = img_pil.resize(new_size, resample=Image.NEAREST)
            # img_resized.save(map_path)

            img_pil.save(map_path, format='PPM') 

            self.get_logger().info("✅ Map saved successfully!")

        except Exception as e:
            self.get_logger().error(f"🔥 Exception during map processing: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MapSaver()
    rclpy.spin_once(node)

if __name__ == '__main__':
    main()

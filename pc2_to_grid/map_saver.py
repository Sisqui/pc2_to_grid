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

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        
        # Declare parameters with default values
        self.declare_parameter("map_topic", "/occupancy_grid_scan")
        self.declare_parameter("map_name", "my_map.png")

        # Get parameter values
        self.map_topic = self.get_parameter("map_topic").get_parameter_value().string_value
        self.map_name = self.get_parameter("map_name").get_parameter_value().string_value

        self.map_subscriber = self.create_subscription(OccupancyGrid, self.map_topic, self.listener_callback, 10)

    def listener_callback(self, msg):
        width, height = msg.info.width, msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin.position

        grid_data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Convert to grayscale image (Unknown=-1, Free=0, Occupied=100)
        image = np.zeros((height, width), dtype=np.uint8)
        image[grid_data == -1] = 128  # Unknown
        image[grid_data == 0] = 255   # Free space
        image[grid_data == 100] = 0   # Occupied

        # Resize to low resolution (e.g., 25% of original size)
        # scale_percent = 95
        # new_width = int(width * scale_percent / 100)
        # new_height = int(height * scale_percent / 100)
        resized_image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)

        # Save the map
        cv2.imwrite(self.map_name, resized_image)
        self.get_logger().info('Saved low-resolution map as my_map.png')

def main(args=None):
    rclpy.init(args=args)
    node = MapSaver()
    rclpy.spin_once(node)

if __name__ == '__main__':
    main()

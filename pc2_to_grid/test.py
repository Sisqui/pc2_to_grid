import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import time

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.gps_publisher = self.create_publisher(Point, '/leakage_gps', 10)
        self.local_publisher = self.create_publisher(Point, '/leakage_local', 10)
        self.timer = self.create_timer(1.0, self.publish_test_data)
        self.count = 0

    def publish_test_data(self):
        gps_point = Point()
        gps_point.x = 2.0 + self.count * 0.1
        gps_point.y = -3.0 + self.count * 0.1
        gps_point.z = 5.0

        local_point = Point()
        local_point.x = 1.0 + self.count * 0.1
        local_point.y = 2.0 + self.count * 0.1
        local_point.z = 4.0

        self.gps_publisher.publish(gps_point)
        self.local_publisher.publish(local_point)
        
        self.get_logger().info(f'Published GPS: {gps_point}')
        self.get_logger().info(f'Published Local: {local_point}')
        
        self.count += 1

def main():
    rclpy.init()
    node = TestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

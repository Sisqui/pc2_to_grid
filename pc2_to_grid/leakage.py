import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

class BatteryRead(Node):
    def __init__(self):
        super().__init__('battery_read')

        # Arrays to store GPS and Local Points
        self.gps_data = []
        self.local_data = []

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            durability=DurabilityPolicy.VOLATILE, 
            depth=10
        )

        # Subscriptions for GPS and Local position
        self.gps_subscription = self.create_subscription(
            Point, '/leakage_gps', self.gps_callback, qos_profile
        )
        self.local_position_subscription = self.create_subscription(
            Point, '/leakage_local', self.local_callback, qos_profile
        )

        # Publisher for MarkerArray
        self.marker_gps_publisher = self.create_publisher(MarkerArray, '/gps_marker', 10)
        self.marker_local_publisher = self.create_publisher(MarkerArray, '/local_marker', 10)

        # Timer to periodically publish markers
        self.timer = self.create_timer(1.0, self.publish_markers)

    def gps_callback(self, msg):
        """Callback function to handle GPS data."""
        self.gps_data.append(msg)  # Store GPS point in the list

    def local_callback(self, msg):
        """Callback function to handle local position data."""
        self.local_data.append(msg)  # Store local point in the list

    def publish_markers(self):
        """Publish two MarkerArrays, one for GPS and one for local position."""
        # Create MarkerArray for GPS
        gps_marker_array = MarkerArray()
        for i, point in enumerate(self.gps_data):
            gps_marker = Marker()
            gps_marker.header.frame_id = "map"
            gps_marker.header.stamp = self.get_clock().now().to_msg()
            gps_marker.ns = "gps_poses"
            gps_marker.id = i
            gps_marker.type = Marker.SPHERE
            gps_marker.action = Marker.ADD
            gps_marker.pose.position.x = point.x
            gps_marker.pose.position.y = point.y
            gps_marker.pose.position.z = point.z
            gps_marker.scale.x = 0.1
            gps_marker.scale.y = 0.1
            gps_marker.scale.z = 0.1
            gps_marker.color.a = 1.0
            gps_marker.color.r = 1.0  # Red for the GPS points
            gps_marker.color.g = 0.0
            gps_marker.color.b = 0.0
            gps_marker_array.markers.append(gps_marker)

        # Publish GPS markers
        self.marker_gps_publisher.publish(gps_marker_array)

        # Create MarkerArray for Local position
        local_marker_array = MarkerArray()
        for i, point in enumerate(self.local_data):
            local_marker = Marker()
            local_marker.header.frame_id = "map"
            local_marker.header.stamp = self.get_clock().now().to_msg()
            local_marker.ns = "local_poses"
            local_marker.id = i
            local_marker.type = Marker.SPHERE
            local_marker.action = Marker.ADD
            local_marker.pose.position.x = point.x
            local_marker.pose.position.y = point.y
            local_marker.pose.position.z = point.z
            local_marker.scale.x = 0.1
            local_marker.scale.y = 0.1
            local_marker.scale.z = 0.1
            local_marker.color.a = 1.0
            local_marker.color.r = 0.0
            local_marker.color.g = 1.0  # Green for the Local points
            local_marker.color.b = 0.0
            local_marker_array.markers.append(local_marker)

        # Publish Local markers
        self.marker_local_publisher.publish(local_marker_array)

def main():
    rclpy.init()
    node = BatteryRead()
    rclpy.spin(node)  # Keep the node running to keep receiving messages
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class BatteryRead(Node):
    def __init__(self):
        super().__init__('battery_read')

        # Store only the latest GPS and Local Point
        self.latest_gps_data = None
        self.latest_local_data = None

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            durability=DurabilityPolicy.VOLATILE, 
            depth=10
        )

        # Subscriptions
        self.gps_subscription = self.create_subscription(
            Point, '/leakage_gps', self.gps_callback, qos_profile
        )
        self.local_subscription = self.create_subscription(
            Point, '/leakage_local', self.local_callback, qos_profile
        )

        # Publishers
        self.gps_marker_publisher = self.create_publisher(Marker, '/gps_marker', 10)
        self.local_marker_publisher = self.create_publisher(Marker, '/local_marker', 10)

        # Timer to publish markers
        self.timer = self.create_timer(1.0, self.publish_markers)

    def gps_callback(self, msg):
        """Store only the latest GPS point."""
        self.latest_gps_data = msg  # Store latest GPS point

    def local_callback(self, msg):
        """Store only the latest Local position point."""
        self.latest_local_data = msg  # Store latest Local point

    def publish_markers(self):
        """Publish the latest GPS and Local position as a single marker each."""
        if self.latest_gps_data:
            gps_marker = Marker()
            gps_marker.header.frame_id = "map"
            gps_marker.header.stamp = self.get_clock().now().to_msg()
            gps_marker.ns = "gps_pose"
            gps_marker.id = 0  # Only one marker, so ID = 0
            gps_marker.type = Marker.SPHERE
            gps_marker.action = Marker.ADD
            gps_marker.pose.position.x = self.latest_gps_data.x
            gps_marker.pose.position.y = self.latest_gps_data.y
            gps_marker.pose.position.z = self.latest_gps_data.z
            gps_marker.scale.x = 0.2
            gps_marker.scale.y = 0.2
            gps_marker.scale.z = 0.2
            gps_marker.color.a = 1.0
            gps_marker.color.r = 1.0  # Red for GPS
            gps_marker.color.g = 0.0
            gps_marker.color.b = 0.0
            self.gps_marker_publisher.publish(gps_marker)

        if self.latest_local_data:
            local_marker = Marker()
            local_marker.header.frame_id = "map"
            local_marker.header.stamp = self.get_clock().now().to_msg()
            local_marker.ns = "local_pose"
            local_marker.id = 0  # Only one marker, so ID = 0
            local_marker.type = Marker.SPHERE
            local_marker.action = Marker.ADD
            local_marker.pose.position.x = self.latest_local_data.x
            local_marker.pose.position.y = self.latest_local_data.y
            local_marker.pose.position.z = self.latest_local_data.z
            local_marker.scale.x = 0.2
            local_marker.scale.y = 0.2
            local_marker.scale.z = 0.2
            local_marker.color.a = 1.0
            local_marker.color.r = 0.0
            local_marker.color.g = 1.0  # Green for Local
            local_marker.color.b = 0.0
            self.local_marker_publisher.publish(local_marker)

def main():

    rclpy.init()
    node = BatteryRead()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

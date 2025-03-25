#!/usr/bin/env python3

import PyKDL
import rclpy
import numpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud, read_points
from tf2_ros import Buffer, TransformListener, TransformRegistration
from geometry_msgs.msg import TransformStamped

class PointCloudTransform(Node):
    """
    A ROS 2 node that subscribes to a PointCloud2 message, transforms it to a specified target frame,
    and republishes the transformed PointCloud2 message.
    """

    def __init__(self):
        super().__init__('pointcloud_transform_node')

        # Declare parameters with default values
        self.declare_parameter("input_topic", "/cloud_in_downsampled")
        self.declare_parameter("output_topic", "/cloud_transformed")
        self.declare_parameter("target_frame", "map")

        # Get parameter values
        self.input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.target_frame = self.get_parameter("target_frame").get_parameter_value().string_value

        # Initialize the TransformListener and Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for the transformed PointCloud2 message
        self.pointcloud_publisher = self.create_publisher(PointCloud2, self.output_topic, 1)

        # Subscriber for the input PointCloud2 message
        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_callback,
            10
        )

    def transform_to_kdl(self, t):
        try:
            return PyKDL.Frame(PyKDL.Rotation.Quaternion(
                                t.transform.rotation.x, t.transform.rotation.y,
                                t.transform.rotation.z, t.transform.rotation.w),
                           PyKDL.Vector(t.transform.translation.x,
                                        t.transform.translation.y,
                                        t.transform.translation.z))
        except Exception as e:
            self.get_logger().error(f"Transform to KDL failed: {str(e)}")

    def do_transform_cloud(self, cloud, transform):
        t_kdl = self.transform_to_kdl(transform)
        points_out = []
        for p_in in read_points(cloud):
            p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
            points_out.append(p_out)
        xyz_fields = [PointField(name='x', offset=0, datatype=7, count=1), PointField(name='y', offset=4, datatype=7, count=1), PointField(name='z', offset=8, datatype=7, count=1)]
        res = create_cloud(transform.header, xyz_fields, points_out)
        return res


    # TransformRegistration().add(PointCloud2, do_transform_cloud)

    def pointcloud_callback(self, msg):
        """
        Callback function for the PointCloud2 message. Transforms the point cloud to the target frame
        and republishes it.
        
        :param msg: Incoming PointCloud2 message.
        """
        try:
            # Look up the transform from the point cloud's frame to the target frame
            # print("Getting transform...")
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time()
            )

            # Transform the point cloud
            # print("Do transform cloud...")
            transformed_cloud = self.do_transform_cloud(msg, transform)

            # Update the header to reflect the new frame
            # print("Updating header...")
            transformed_cloud.header.frame_id = self.target_frame
            # transformed_cloud.header.stamp = self.get_clock().now().to_msg()
            transformed_cloud.header.stamp = msg.header.stamp

            # Publish the transformed point cloud
            # print("Publishing...\n\n")
            self.pointcloud_publisher.publish(transformed_cloud)

        except Exception as e:
            self.get_logger().error(f"Transform failed: {str(e)}")

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create the node
    node = PointCloudTransform()
    
    # Keep the node running
    rclpy.spin(node)
    
    # Shutdown the node when done
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

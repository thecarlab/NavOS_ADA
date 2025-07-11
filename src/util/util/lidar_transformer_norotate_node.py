import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
import laser_geometry.laser_geometry as lg
import numpy as np
from rclpy.qos import qos_profile_sensor_data


class LidarTransformerNode(Node):
    def __init__(self):
        super().__init__('lidar_transformer_node')
        self.laser_projector = lg.LaserProjection()
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(PointCloud2, '/cloud_in', qos_profile_sensor_data)
        
        self.get_logger().info('Lidar Transformer Node has started.')

    def scan_callback(self, scan_msg):
        try:
            cloud_msg = self.laser_projector.projectLaser(scan_msg)
            
            cloud_msg.header.frame_id = "base_link"
            self.publisher.publish(cloud_msg)

        except Exception as e:
            self.get_logger().warn(f"Laser projection failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = LidarTransformerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

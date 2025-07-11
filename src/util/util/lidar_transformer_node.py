import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
import laser_geometry.laser_geometry as lg
import numpy as np
import struct
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
        self.publisher = self.create_publisher(
            PointCloud2,
            '/cloud_in',
            qos_profile_sensor_data
        )

        self.get_logger().info('Lidar Transformer Node has started.')

    def scan_callback(self, scan_msg):
        try:
            # 1. Project LaserScan to PointCloud2 (includes intensity if present in scan_msg)
            cloud_msg = self.laser_projector.projectLaser(scan_msg)

            # 2. Unpack PointCloud2 to x, y, z, intensity
            points = self.pointcloud2_to_xyz_intensity(cloud_msg)
            if points.size == 0:
                self.get_logger().warn("No valid points in point cloud.")
                return

            # 3. Rotate 180 degrees around Z (negate X and Y)
            rotated_points = points.copy()
            rotated_points[:, 0] *= -1  # X
            rotated_points[:, 1] *= -1  # Y

            # 4. Pack and publish new PointCloud2
            rotated_cloud_msg = self.xyz_intensity_to_pointcloud2(rotated_points, cloud_msg.header)
            rotated_cloud_msg.header.frame_id = "base_link"

            self.publisher.publish(rotated_cloud_msg)

        except Exception as e:
            self.get_logger().warn(f"Error in scan_callback: {str(e)}")

    def pointcloud2_to_xyz_intensity(self, cloud_msg):
        """Convert PointCloud2 binary buffer to Nx4 numpy array: x, y, z, intensity."""
        if cloud_msg.point_step == 0 or not cloud_msg.data:
            return np.empty((0, 4), dtype=np.float32)

        # Locate field offsets
        field_offsets = {f.name: f.offset for f in cloud_msg.fields}
        for required_field in ['x', 'y', 'z', 'intensity']:
            if required_field not in field_offsets:
                raise ValueError(f"PointCloud2 missing {required_field} field")

        num_points = cloud_msg.width * cloud_msg.height
        all_points = []

        for i in range(num_points):
            offset = i * cloud_msg.point_step
            x = struct.unpack_from('f', cloud_msg.data, offset + field_offsets['x'])[0]
            y = struct.unpack_from('f', cloud_msg.data, offset + field_offsets['y'])[0]
            z = struct.unpack_from('f', cloud_msg.data, offset + field_offsets['z'])[0]
            intensity = struct.unpack_from('f', cloud_msg.data, offset + field_offsets['intensity'])[0]
            if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                all_points.append([x, y, z, intensity])

        return np.array(all_points, dtype=np.float32)

    def xyz_intensity_to_pointcloud2(self, points, header):
        """Pack Nx4 numpy array back into PointCloud2."""
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = points.shape[0]
        msg.is_dense = True
        msg.is_bigendian = False
        msg.point_step = 16  # 4x float32
        msg.row_step = msg.point_step * msg.width

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        buffer = []
        for p in points:
            buffer.append(struct.pack('ffff', p[0], p[1], p[2], p[3]))
        msg.data = b''.join(buffer)

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = LidarTransformerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


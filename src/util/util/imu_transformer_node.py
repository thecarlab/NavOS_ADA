#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_matrix, quaternion_from_matrix, quaternion_multiply
from rclpy.qos import qos_profile_sensor_data

class ImuTransformerNode(Node):
    def __init__(self):
        super().__init__('imu_transformer_node')

        self.subscriber = self.create_subscription(
            Imu,
            '/camera/camera/imu',
            self.imu_callback,
            qos_profile_sensor_data
        )

        self.publisher = self.create_publisher(
            Imu,
            '/imu_transformed',
            qos_profile_sensor_data
        )

        self.get_logger().info('IMU Transformer Node has started.')

        # Define rotation matrix for coordinate transform
        # base_link = R * camera_imu_optical
        # base_link.x = -camera.z
        # base_link.y = -camera.x
        # base_link.z = camera.y
        self.rotation_matrix = np.array([
            [ 0,  0, -1],
            [-1,  0,  0],
            [ 0, 1,  0]
        ])

        # Precompute rotation quaternion
        self.rotation_quaternion = self.rotation_matrix_to_quaternion(self.rotation_matrix)

    def rotation_matrix_to_quaternion(self, R):
        M = np.eye(4)
        M[:3, :3] = R
        q = quaternion_from_matrix(M)
        return q

    def transform_vector(self, v):
        v_np = np.array([[v.x], [v.y], [v.z]])
        v_transformed = np.dot(self.rotation_matrix, v_np)
        return v_transformed.flatten()

    def imu_callback(self, msg):
        transformed_msg = Imu()

        # Header
        transformed_msg.header.stamp = msg.header.stamp
        transformed_msg.header.frame_id = 'base_link'

        # Transform orientation
        q_in = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]

        q_rot = self.rotation_quaternion
        q_out = quaternion_multiply(q_rot, q_in)

        transformed_msg.orientation = Quaternion(
            x=q_out[0],
            y=q_out[1],
            z=q_out[2],
            w=q_out[3]
        )
        transformed_msg.orientation_covariance = msg.orientation_covariance

        # Transform angular_velocity
        av = self.transform_vector(msg.angular_velocity)
        transformed_msg.angular_velocity.x = av[0]
        transformed_msg.angular_velocity.y = av[1]
        transformed_msg.angular_velocity.z = av[2]
        transformed_msg.angular_velocity_covariance = msg.angular_velocity_covariance

        # Transform linear_acceleration
        la = self.transform_vector(msg.linear_acceleration)
        transformed_msg.linear_acceleration.x = la[0]
        transformed_msg.linear_acceleration.y = la[1]
        transformed_msg.linear_acceleration.z = la[2]
        transformed_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance

        # Publish
        self.publisher.publish(transformed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuTransformerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


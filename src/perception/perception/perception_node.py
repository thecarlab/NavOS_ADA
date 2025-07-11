#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # Create CvBridge
        self.bridge = CvBridge()

        # Load YOLO model
        self.model = YOLO("/perception_ws/src/models/yolo11n.engine")

        # Declare parameters for topic names
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("inference_image_topic", "/yolo/inference_image")
        self.declare_parameter("detection_topic", "/yolo/detections")

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        inference_image_topic = self.get_parameter("inference_image_topic").get_parameter_value().string_value
        detection_topic = self.get_parameter("detection_topic").get_parameter_value().string_value

        # Subscribers
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        # Publishers
        self.image_publisher = self.create_publisher(
            Image,
            inference_image_topic,
            10
        )
        self.detection_publisher = self.create_publisher(
            String,
            detection_topic,
            10
        )

        self.get_logger().info(
            f"YOLO Inference Node Started. Subscribed to {image_topic}, "
            f"publishing annotated image to {inference_image_topic} and detections to {detection_topic}"
        )

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge exception: {e}")
            return

        # Run YOLO inference
        results = self.model(cv_image)

        if results:
            # 1️⃣ Publish annotated image for visualization
            annotated_image = results[0].plot()
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            self.image_publisher.publish(annotated_msg)
            #self.get_logger().info("Published annotated inference image.")

            # 2️⃣ Prepare detection text message for control logic
            detected_info = []
            for result in results:
                if result.boxes and result.boxes.cls is not None:
                    cls_ids = result.boxes.cls.cpu().numpy()
                    bboxes = result.boxes.xyxy.cpu().numpy()
                    confs = result.boxes.conf.cpu().numpy()

                    for cls_id, bbox, conf in zip(cls_ids, bboxes, confs):
                        class_name = self.model.names[int(cls_id)]
                        x1, y1, x2, y2 = bbox
                        detected_info.append(
                            f"{class_name} [{x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f}] ({conf:.2f})"
                        )

            if detected_info:
                detection_msg = "; ".join(detected_info)
            else:
                detection_msg = "no detections"

            self.detection_publisher.publish(String(data=detection_msg))
            #self.get_logger().info(f"Published detections: {detection_msg}")

        else:
            self.get_logger().info("No detections found in this frame.")
            self.detection_publisher.publish(String(data="no detections"))


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


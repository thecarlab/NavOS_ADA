#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Header, String

class StopSignBehaviorNode(Node):

    def __init__(self):
        super().__init__('stop_sign_behavior_node')

        # Parameters
        self.declare_parameter('stop_duration_sec', 3.0)
        self.declare_parameter('approach_speed', 0.5)
        self.declare_parameter('cmd_topic', '/stopsign_cmd')
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('frame_threshold', 10)  # frames needed to confirm state change

        self.stop_duration = self.get_parameter('stop_duration_sec').value
        self.approach_speed = self.get_parameter('approach_speed').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.frame_threshold = self.get_parameter('frame_threshold').value

        # State
        self.stop_sign_frame_count = 0  # Count of consecutive frames with stop sign
        self.no_stop_sign_frame_count = 0  # Count of consecutive frames without stop sign
        self.in_stop_procedure = False
        self.stop_start_time = None
        self.completed_stop = False  # Flag to indicate if we've completed the stop procedure
        self.found_stop_sign = False
        self.found_no_stop_sign = False
        self.completed_approach = False  # Flag to track if we've completed the approach phase

        # Subscribers
        self.create_subscription(
            String,
            self.detections_topic,
            self.detections_callback,
            10
        )

        # Publisher
        self.cmd_pub = self.create_publisher(
            AckermannDriveStamped,
            self.cmd_topic,
            10
        )

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Stop Sign Behavior Node Started")

    def detections_callback(self, msg):
        if self.in_stop_procedure:
            return

        # Parse all detections in the message
        detections = msg.data.split(';')
        found_valid_stop_sign = False

        # Check each detection
        for detection in detections:
            if not detection.strip():  # Skip empty detections
                continue
                
            # Parse detection: "class_name [x1, y1, x2, y2] (conf)"
            parts = detection.strip().split()
            if len(parts) < 2:  # Skip malformed detections
                continue

            # Find class name (might be multiple words like "stop sign")
            box_start = detection.find('[')
            if box_start == -1:
                continue
            class_name = detection[:box_start].strip()
            
            # Find confidence
            conf_start = detection.rfind('(')
            conf_end = detection.rfind(')')
            if conf_start == -1 or conf_end == -1:
                continue
            try:
                confidence = float(detection[conf_start+1:conf_end])
            except ValueError:
                continue

            # Check if it's a stop sign with high confidence
            if class_name == "stop sign" and confidence > 0.88:
                found_valid_stop_sign = True
                break

        # Update frame counts based on detection result
        if found_valid_stop_sign:
            self.stop_sign_frame_count += 1
            self.no_stop_sign_frame_count = 0
        else:
            self.no_stop_sign_frame_count += 1
            self.stop_sign_frame_count = 0

        # Update detection states if we have stable detection
        if self.stop_sign_frame_count >= self.frame_threshold:
            if not self.found_stop_sign:
                self.get_logger().info("Stable stop sign detection")
            self.found_stop_sign = True
            self.found_no_stop_sign = False
        elif self.no_stop_sign_frame_count >= self.frame_threshold:
            if not self.found_no_stop_sign:
                self.get_logger().info("Stable no stop sign detection")
            self.found_no_stop_sign = True
            self.found_stop_sign = False
        
        # Log current state
        self.get_logger().debug(f"Stop frames: {self.stop_sign_frame_count}, No stop frames: {self.no_stop_sign_frame_count}")

    def control_loop(self):
        #self.get_logger().debug("Control Loop Triggered")
        #self.get_logger().debug("Stop sign frame count: %d" % self.stop_sign_frame_count)
        #self.get_logger().debug("No stop sign frame count: %d" % self.no_stop_sign_frame_count)

        # If we're already in stop procedure, handle the stop timing
        if self.in_stop_procedure:
            if time.time() - self.stop_start_time >= self.stop_duration:
                self.get_logger().info("Stop complete, stopping all commands")
                self.in_stop_procedure = False
                # Reset all counters and states after stop is complete
                self.stop_sign_frame_count = 0
                self.no_stop_sign_frame_count = 0
                self.found_stop_sign = False
                self.found_no_stop_sign = False
                self.completed_approach = False
            else:
                stop_cmd = self.create_stop_command()
                self.cmd_pub.publish(stop_cmd)
            return

        # If we have a stable stop sign detection, continuously publish approach commands
        if self.found_stop_sign:
            self.get_logger().info("Slow down!")
            self.completed_approach = True  # Mark that we've done the approach phase
            #approach_cmd = self.create_approach_command()
            #self.cmd_pub.publish(approach_cmd)
        # Only initiate stop if we've completed approach and now have stable no-stop-sign detection
        elif self.found_no_stop_sign and self.completed_approach and not self.in_stop_procedure:
            self.get_logger().info("Finished approach and passed stop sign - initiating stop")
            self.in_stop_procedure = True
            self.stop_start_time = time.time()
            stop_cmd = self.create_stop_command()
            self.cmd_pub.publish(stop_cmd)

    def create_stop_command(self):
        cmd = AckermannDriveStamped()
        cmd.header = Header()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.drive.speed = 0.0
        cmd.drive.steering_angle = 0.0
        return cmd

    def create_approach_command(self):
        cmd = AckermannDriveStamped()
        cmd.header = Header()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.drive.speed = self.approach_speed
        cmd.drive.steering_angle = 0.0
        return cmd


def main(args=None):
    rclpy.init(args=args)
    node = StopSignBehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


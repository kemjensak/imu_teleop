#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformListener
from tf2_ros import Buffer
import tf2_py
import quaternion

class IMURelativeOrientationNode(Node):
    def __init__(self):
        super().__init__('imu_relative_orientation_node')

        self.declare_parameter('waist_imu_id', rclpy.Parameter.Type.STRING) 
        self.declare_parameter('shoulder_imu_id', rclpy.Parameter.Type.STRING)    
        self.declare_parameter('elbow_imu_id', rclpy.Parameter.Type.STRING)
        self.declare_parameter('wrist_imu_id', rclpy.Parameter.Type.STRING)

        (waist_imu_id, shoulder_imu_id, elbow_imu_id, wrist_imu_id) = self.get_parameters(
            ['waist_imu_id', 'shoulder_imu_id', 'elbow_imu_id', 'wrist_imu_id'])

        # Subscribers for the IMU data
        self.waist_sub = self.create_subscription(Imu, '/imu_' + waist_imu_id.value, self.imu_waist_callback, 1)
        self.shoulder_sub = self.create_subscription(Imu, '/imu_' + shoulder_imu_id.value, self.imu_shoulder_callback, 1)
        self.elbow_sub = self.create_subscription(Imu, '/imu_' + elbow_imu_id.value, self.imu_elbow_callback, 1)
        self.wrist_sub = self.create_subscription(Imu, '/imu_' + wrist_imu_id.value, self.imu_wrist_callback, 1)

        # Publishers for the processed IMU data
        self.processed_waist_pub = self.create_publisher(Imu, '/calibrated_imu_waist/data', 1)
        self.processed_shoulder_pub = self.create_publisher(Imu, '/calibrated_imu_shoulder/data', 1)
        self.processed_elbow_pub = self.create_publisher(Imu, '/calibrated_imu_elbow/data', 1)
        self.processed_wrist_pub = self.create_publisher(Imu, '/calibrated_imu_wrist/data', 1)

        # Placeholders for initial IMU data and relative orientations
        self.initial_imu_data_received = False
        self.initial_waist_orientation = None
        self.initial_shoulder_orientation = None
        self.initial_elbow_orientation = None
        self.initial_wrist_orientation = None
        self.relative_shoulder_orientation = None
        self.relative_elbow_orientation = None
        self.relative_wrist_orientation = None
        self.imu_shoulder = None
        self.imu_elbow = None
        self.imu_waist = None

    def imu_waist_callback(self, msg):
        if not self.initial_imu_data_received:
            self.initial_waist_orientation = msg.orientation
        self.imu_waist_orientation = msg.orientation
        self.imu_waist = msg
        self.process_initial_data()

    def imu_shoulder_callback(self, msg):
        if not self.initial_imu_data_received:
            self.initial_shoulder_orientation = msg.orientation
        self.imu_shoulder_orientation = msg.orientation
        self.imu_shoulder = msg
        self.process_initial_data()

    def imu_elbow_callback(self, msg):
        if not self.initial_imu_data_received:
            self.initial_elbow_orientation = msg.orientation
        self.imu_elbow_orientation = msg.orientation
        self.imu_elbow = msg
        self.process_initial_data()

    def imu_wrist_callback(self, msg):
        if not self.initial_imu_data_received:
            self.initial_wrist_orientation = msg.orientation
        self.imu_wrist_orientation = msg.orientation
        self.imu_wrist = msg
        self.process_initial_data()

    def process_initial_data(self):
        # Check if initial data is received for all IMUs and calculate relative orientations only once
        if not self.initial_imu_data_received and self.initial_waist_orientation and self.initial_shoulder_orientation and self.initial_elbow_orientation and self.initial_wrist_orientation:
            # Calculate fixed relative orientations
            self.relative_shoulder_orientation = self.calculate_relative_orientation(self.initial_waist_orientation, self.initial_shoulder_orientation)
            self.relative_elbow_orientation = self.calculate_relative_orientation(self.initial_waist_orientation, self.initial_elbow_orientation)
            self.relative_wrist_orientation = self.calculate_relative_orientation(self.initial_waist_orientation, self.initial_wrist_orientation)
            self.initial_imu_data_received = True

        # Process and publish new data only after initial relative orientations are set
        if self.initial_imu_data_received:
            self.process_and_publish_imu_shoulder()
            self.process_and_publish_imu_elbow()
            self.process_and_publish_imu_waist()
            self.process_and_publish_imu_wrist()

    def process_and_publish_imu_waist(self):
        # Process and publish waist data
        if self.imu_waist_orientation:
            processed_imu_data = Imu()
            processed_imu_data = self.imu_waist
            processed_imu_data.header.frame_id = "world"
            processed_imu_data.orientation = self.imu_waist_orientation
            self.processed_waist_pub.publish(processed_imu_data)
    
    def process_and_publish_imu_shoulder(self):
        # Process and publish shoulder data
        if self.imu_waist_orientation and self.imu_shoulder_orientation:
            processed_shoulder_orientation = self.calculate_relative_orientation(self.relative_shoulder_orientation, self.imu_shoulder_orientation)
            processed_imu_data = Imu()
            processed_imu_data = self.imu_shoulder
            processed_imu_data.header.frame_id = "world"
            processed_imu_data.orientation = processed_shoulder_orientation
            self.processed_shoulder_pub.publish(processed_imu_data)

    def process_and_publish_imu_elbow(self):
        # Process and publish elbow data
        if self.imu_waist_orientation and self.imu_elbow_orientation:
            processed_elbow_orientation = self.calculate_relative_orientation(self.relative_elbow_orientation, self.imu_elbow_orientation)
            processed_imu_data = Imu()
            processed_imu_data = self.imu_elbow
            processed_imu_data.header.frame_id = "world"
            processed_imu_data.orientation = processed_elbow_orientation
            self.processed_elbow_pub.publish(processed_imu_data)

    def process_and_publish_imu_wrist(self):
        # Process and publish wrist data
        if self.imu_waist_orientation and self.imu_wrist_orientation:
            processed_wrist_orientation = self.calculate_relative_orientation(self.relative_wrist_orientation, self.imu_wrist_orientation)
            processed_imu_data = Imu()
            processed_imu_data = self.imu_wrist
            processed_imu_data.header.frame_id = "world"
            processed_imu_data.orientation = processed_wrist_orientation
            self.processed_wrist_pub.publish(processed_imu_data)

    def calculate_relative_orientation(self, ref_orientation, target_orientation):
        # Convert ROS Quaternion message to numpy-quaternion
        ref_quat = quaternion.from_float_array([ref_orientation.w, ref_orientation.x, ref_orientation.y, ref_orientation.z])
        target_quat = quaternion.from_float_array([target_orientation.w, target_orientation.x, target_orientation.y, target_orientation.z])

        # Calculate relative orientation
        relative_quat = ref_quat.conjugate() * target_quat

        # Convert back to Quaternion message
        relative_orientation = Quaternion()
        relative_orientation.w, relative_orientation.x, relative_orientation.y, relative_orientation.z = quaternion.as_float_array(relative_quat)
        return relative_orientation


def main(args=None):
    rclpy.init(args=args)
    node = IMURelativeOrientationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

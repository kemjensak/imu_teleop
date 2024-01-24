#!/usr/bin/python3
import sys
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
from ament_index_python.packages import get_package_share_directory
import quaternion
from rclpy.duration import Duration
import threading
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply, quaternion_matrix


class fbfTransformer(Node):
    def __init__(self):
        super().__init__('fbf_imu')
        self.get_bodyfixed_rotation_matrix()
        self.sub_elbow = self.create_subscription(
            Imu, '/calibrated_imu_elbow/data', self.elbow_imu_callback, 1)
        self.sub_waist = self.create_subscription(
            Imu, '/calibrated_imu_waist/data', self.waist_imu_callback, 1)
        self.sub_shoulder = self.create_subscription(
            Imu, '/calibrated_imu_shoulder/data', self.shoulder_imu_callback, 1)
        self.sub_wrist = self.create_subscription(
            Imu, '/calibrated_imu_wrist/data', self.wrist_imu_callback, 1)
        self.pub_target_pose = self.create_publisher(
            PoseStamped, 'imu_teleop_pose', 1)
        
        self.create_timer(0.01, self.main)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.elbow_quat = None
        self.shoulder_quat = None
        self.waist_quat = None
        self.wrist_quat = None
        self.initial_elbow_quat = None
        self.initial_shoulder_quat = None
        self.initial_waist_quat = None
        self.initial_wrist_quat = None
        self.heading_tf = None

        self.pub_ori = self.create_publisher(
            Float64MultiArray, 'ori_msg', 1)

    def get_bodyfixed_rotation_matrix(self):

        yaml_file_path = os.path.join(get_package_share_directory('imu_teleop'), 'config', 'bodyfixed_quat.yaml')

        with open(yaml_file_path, 'r') as file:
            lines = file.readlines()
            bodyfixed_quat_readline = lines[0].replace('bodyfixed_quat: ', '')
            self.bodyfixed_quat = np.array(eval(bodyfixed_quat_readline.strip()))
            self.bodyfixed_quat = quaternion.from_float_array(self.bodyfixed_quat)
        print(self.bodyfixed_quat)

    def elbow_imu_callback(self, msg):
        quat_msg = quaternion.from_float_array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])
        if self.initial_elbow_quat is None:
            self.initial_elbow_quat = quat_msg
        self.elbow_quat = quat_msg

    def shoulder_imu_callback(self, msg):
        quat_msg = quaternion.from_float_array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])
        if self.initial_shoulder_quat is None:
            self.initial_shoulder_quat = quat_msg
        self.shoulder_quat = quat_msg

    def waist_imu_callback(self, msg):
        quat_msg = quaternion.from_float_array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])
        if self.initial_waist_quat is None:
            self.initial_waist_quat = quat_msg
        self.waist_quat = quat_msg

    def wrist_imu_callback(self, msg):
        quat_msg = quaternion.from_float_array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])
        if self.initial_wrist_quat is None:
            self.initial_wrist_quat = quat_msg
        self.wrist_quat = quat_msg

    def broadcast_tf(self, parent_frame, child_frame, translation, rotation):
        time_msg = self.get_clock().now().to_msg()
        tf = tf2_ros.TransformStamped()
        tf.header.stamp = time_msg
        tf.header.frame_id = parent_frame
        tf.child_frame_id = child_frame
        tf.transform.translation.x = translation[0]
        tf.transform.translation.y = translation[1]
        tf.transform.translation.z = translation[2]
        tf.transform.rotation.x = rotation[0]
        tf.transform.rotation.y = rotation[1]
        tf.transform.rotation.z = rotation[2]
        tf.transform.rotation.w = rotation[3]
        self.tf_broadcaster.sendTransform(tf)


    def get_tf(self, parent_frame, child_frame):
        try:
            t = self.tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                rclpy.time.Time(),
            )
            return t
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform : {ex}')
            return None

        
    def calc_quat(self, waist_quat, shoulder_quat, elbow_quat, wrist_quat):
        if waist_quat is None or shoulder_quat is None or elbow_quat is None or wrist_quat is None:
            return None

        self.c_shoulder_quat = self.initial_shoulder_quat.conjugate() * self.bodyfixed_quat
        self.c_waist_quat = self.initial_waist_quat.conjugate() *  self.bodyfixed_quat
        self.c_elbow_quat = self.initial_elbow_quat.conjugate() * self.bodyfixed_quat
        self.c_wrist_quat = self.initial_wrist_quat.conjugate() * self.bodyfixed_quat

        fbf_quat = waist_quat * self.c_waist_quat

        shoulder_ing_quat = fbf_quat.conjugate() * shoulder_quat
        shoulder_moving_quat = shoulder_ing_quat * self.c_shoulder_quat

        elbow_ing_quat = fbf_quat.conjugate() * elbow_quat
        elbow_moving_quat = elbow_ing_quat * self.c_wrist_quat

        wrist_ing_quat = fbf_quat.conjugate() * wrist_quat
        wrist_moving_quat = wrist_ing_quat * self.c_elbow_quat



        shoulder_to_elbow_quat = shoulder_moving_quat.conjugate() * elbow_moving_quat
        elbow_to_wrist_quat = elbow_moving_quat.conjugate() * wrist_moving_quat
        
        fbf_quat = quaternion.as_float_array(fbf_quat)
        shoulder_quat = quaternion.as_float_array(shoulder_moving_quat)
        elbow_quat = quaternion.as_float_array(shoulder_to_elbow_quat)
        wrist_quat = quaternion.as_float_array(elbow_to_wrist_quat)

        time_msg = self.get_clock().now().to_msg()

        print(fbf_quat)
        tf_fbf = tf2_ros.TransformStamped()
        tf_fbf.header.stamp = time_msg
        tf_fbf.header.frame_id = "world"
        tf_fbf.child_frame_id = "fbf"
        tf_fbf.transform.translation.x = 0.0
        tf_fbf.transform.translation.y = 0.0
        tf_fbf.transform.translation.z = 1.0
        tf_fbf.transform.rotation.x = fbf_quat[1]
        tf_fbf.transform.rotation.y = fbf_quat[2]
        tf_fbf.transform.rotation.z = fbf_quat[3]
        tf_fbf.transform.rotation.w = fbf_quat[0]
        self.tf_broadcaster.sendTransform(tf_fbf)

        tf_shoulder = tf2_ros.TransformStamped()
        tf_shoulder.header.stamp = time_msg
        tf_shoulder.header.frame_id = "fbf"
        tf_shoulder.child_frame_id = "shoulder"
        tf_shoulder.transform.translation.x = 0.0
        tf_shoulder.transform.translation.y = -0.3
        tf_shoulder.transform.translation.z = 0.0
        tf_shoulder.transform.rotation.x = shoulder_quat[1]
        tf_shoulder.transform.rotation.y = shoulder_quat[2]
        tf_shoulder.transform.rotation.z = shoulder_quat[3]
        tf_shoulder.transform.rotation.w = shoulder_quat[0]
        self.tf_broadcaster.sendTransform(tf_shoulder)

        tf_elbow = tf2_ros.TransformStamped()
        tf_elbow.header.stamp = time_msg
        tf_elbow.header.frame_id = "shoulder"
        tf_elbow.child_frame_id = "elbow"
        tf_elbow.transform.translation.x = 0.0
        tf_elbow.transform.translation.y = 0.0
        tf_elbow.transform.translation.z = -0.27
        tf_elbow.transform.rotation.x = elbow_quat[1]
        tf_elbow.transform.rotation.y = elbow_quat[2]
        tf_elbow.transform.rotation.z = elbow_quat[3]
        tf_elbow.transform.rotation.w = elbow_quat[0]
        self.tf_broadcaster.sendTransform(tf_elbow)

        tf_wrist = tf2_ros.TransformStamped()
        tf_wrist.header.stamp = time_msg
        tf_wrist.header.frame_id = "elbow"
        tf_wrist.child_frame_id = "wrist"
        tf_wrist.transform.translation.x = 0.0
        tf_wrist.transform.translation.y = 0.0
        tf_wrist.transform.translation.z = -0.3
        tf_wrist.transform.rotation.x = wrist_quat[1]
        tf_wrist.transform.rotation.y = wrist_quat[2]
        tf_wrist.transform.rotation.z = -wrist_quat[3]
        tf_wrist.transform.rotation.w = wrist_quat[0]
        self.tf_broadcaster.sendTransform(tf_wrist)

        robot_heading_tf = self.get_tf("base_link", "shoulder_link")
        if robot_heading_tf is not None:
            fake_heading_tf = robot_heading_tf
            fake_heading_tf.header.stamp = time_msg
            fake_heading_tf.header.frame_id = "fbf"
            fake_heading_tf.child_frame_id = "inversed_robot_heading"
            fake_heading_tf.transform.translation.x = 0.0
            fake_heading_tf.transform.translation.y = 0.0
            fake_heading_tf.transform.translation.z = 0.0
            fake_heading_tf.transform.rotation.w *= -1
            self.tf_broadcaster.sendTransform(fake_heading_tf)

        target_pose_tf = self.get_tf("inversed_robot_heading", "wrist")
        if target_pose_tf is not None:
            target_pose = PoseStamped()
            target_pose.header.stamp = time_msg
            target_pose.header.frame_id = "base_link"
            target_pose.pose.position.x = target_pose_tf.transform.translation.x
            target_pose.pose.position.y = target_pose_tf.transform.translation.y
            target_pose.pose.position.z = target_pose_tf.transform.translation.z
            target_pose.pose.orientation.x = target_pose_tf.transform.rotation.x
            target_pose.pose.orientation.y = target_pose_tf.transform.rotation.y
            target_pose.pose.orientation.z = target_pose_tf.transform.rotation.z
            target_pose.pose.orientation.w = target_pose_tf.transform.rotation.w
            self.pub_target_pose.publish(target_pose)
        

        
            
        # self.data_pub()
            
    def main(self):
        self.calc_quat(self.waist_quat, self.shoulder_quat, self.elbow_quat, self.wrist_quat)
        
        
    def data_pub(self):
        pub_ori_msg = Float64MultiArray()
        pub_ori_msg.data = [
            self.s_roll, self.s_pitch, self.s_yaw, self.w2_pitch, self.w2_roll, self.w2_yaw]
        self.pub_ori.publish(pub_ori_msg)

        pub_ang_msg = Float64MultiArray()
        pub_ang_msg.data = [
            self.s_ang_x, self.s_ang_y, self.s_ang_z, self.w_ang_x, self.w_ang_y, self.w_ang_z, self.fbf_ang_x, self.fbf_ang_y, self.fbf_ang_z]
        self.pub_ang.publish(pub_ang_msg)

        pub_pure_msg = Float64MultiArray()
        pub_pure_msg.data = [
            self.pure_s_ang_x, self.pure_s_ang_y, self.pure_s_ang_z, self.pure_w_ang_x, self.pure_w_ang_y, self.pure_w_ang_z]
        self.pub_pure_ang.publish(pub_pure_msg)

    

if __name__ == '__main__':
    rclpy.init()
    fbfTransformer = fbfTransformer()
    rclpy.spin(fbfTransformer)
    # thread = threading.Thread(target=rclpy.spin, args=(fbfTransformer, ), daemon=True)
    # thread.start()
    # fbfTransformer.main()
    # rclpy.spin(euler_data_printer)

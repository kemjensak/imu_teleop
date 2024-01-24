#!/usr/bin/env python3

import sys
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf_transformations import euler_from_quaternion, quaternion_from_matrix
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from ament_index_python.packages import get_package_share_directory
import numpy as np
from math import degrees
import quaternion

stand_time = 5  # seconds
rest_time = 3  # seconds
stoop_time = 5  # seconds

class calc_bf_quat(Node):
    def __init__(self):
        super().__init__('calc_bf_quat')
        self.sub = self.create_subscription(Imu, '/imu_00B4BAEF', self.imu_callback, 1)
        self.imu_data = None
        self.stand_data = self.stoop_data = np.empty((0,3), dtype=np.double)   
        self.data_num = 0

    def imu_callback(self, msg):
        self.imu_data = msg

    def perform_calculation(self, data:Imu):
        if data is not None:
            mean_value = np.mean(data, axis=0)
            return mean_value
        else:
            return None
        
    def imu_to_rpy(self, data:Imu) -> np.ndarray:
        if data is not None:
            orientation = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientation)
            return np.array([[roll, pitch, yaw]])
        else:
            return None
        
    def stack_rpy_data(self, data:tuple, posture:String) -> np.ndarray:
        if data is not None:
            if posture == "stand":
                self.stand_data = np.append(self.stand_data, data, axis=0)
            if posture == "stoop":
                self.stoop_data = np.append(self.stoop_data, data, axis=0)
        else:
            self.get_logger().error("Abnormal RPY data")

    def set_bodyfixed_mat(self):
        stand_mean = np.mean(self.stand_data, axis=0)
        stand_mean[0] = degrees(stand_mean[0])
        stand_mean[1] = degrees(stand_mean[1])
        stand_mean[2] = degrees(stand_mean[2])
        stoop_mean = np.mean(self.stoop_data, axis=0)
        stoop_mean[0] = degrees(stoop_mean[0])
        stoop_mean[1] = degrees(stoop_mean[1])
        stoop_mean[2] = degrees(stoop_mean[2])
     
        self.stand_rotation_matrix = self.euler_to_rotation_matrix(
            stand_mean[0], stand_mean[1], stand_mean[2])

        self.stoop_rotation_matrix = self.euler_to_rotation_matrix(
            stoop_mean[0], stoop_mean[1], stoop_mean[2])

        k_rotation_matrix = np.dot(self.stoop_rotation_matrix, self.stand_rotation_matrix.T)
        trace_R = np.trace(k_rotation_matrix)
        k_angle = np.arccos((trace_R - 1) / 2)

        epsilon = 1e-6
        if k_angle < epsilon:
            k_axis = np.array([1, 0, 0])
        else:
            rx = k_rotation_matrix[2, 1] - k_rotation_matrix[1, 2]
            ry = k_rotation_matrix[0, 2] - k_rotation_matrix[2, 0]
            rz = k_rotation_matrix[1, 0] - k_rotation_matrix[0, 1]
            k_axis = np.array([rx, ry, rz])
            k_axis = k_axis / np.linalg.norm(k_axis)

        print("k", k_axis)

        z_axis = np.array([0, 0, 1])

        x_axis = np.cross(k_axis, z_axis)
        y_axis = np.cross(z_axis, x_axis)

        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        z_axis = z_axis / np.linalg.norm(z_axis)

        assert np.isclose(np.dot(x_axis, y_axis), 0) and np.isclose(np.dot(y_axis, z_axis), 0) and np.isclose(
            np.dot(z_axis, x_axis), 0), "Vectors must be orthogonal"

        bf_rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))
        # self.quat = quaternion_from_matrix(bf_rotation_matrix)
        bodyfixed_quat = quaternion.from_rotation_matrix(bf_rotation_matrix)
        bodyfixed_float = quaternion.as_float_array(bodyfixed_quat)

        yaml_file_path = os.path.join(get_package_share_directory('imu_teleop'), 'config', 'bodyfixed_quat.yaml')
        with open(yaml_file_path, 'w') as f:
            f.write("bodyfixed_quat: " + str(bodyfixed_float[0]) + "," + str(bodyfixed_float[1]) + "," + str(bodyfixed_float[2]) + "," + str(bodyfixed_float[3])  + "\n")
            f.close()

        print("Rotation Matrix:")
        print(bf_rotation_matrix)
        print(bodyfixed_float)

    def euler_to_rotation_matrix(self, roll, pitch, yaw):
        """
        Convert Euler angles (in degrees) to a rotation matrix in ZYX order.
        
        Parameters:
        - yaw: Rotation angle in degrees around the Z-axis
        - pitch: Rotation angle in degrees around the Y-axis
        - roll: Rotation angle in degrees around the X-axis
        
        Returns:
        - A 3x3 rotation matrix.
        """
        
        # Convert to radians
        yaw_rad = np.radians(yaw)
        pitch_rad = np.radians(pitch)
        roll_rad = np.radians(roll)
        
        # Pre-calculate sine and cosine for each angle
        sz = np.sin(yaw_rad)
        cz = np.cos(yaw_rad)
        sy = np.sin(pitch_rad)
        cy = np.cos(pitch_rad)
        sx = np.sin(roll_rad)
        cx = np.cos(roll_rad)
        
        # Define the rotation matrix using the ZYX Euler angles
        r00 = cz * cy
        r01 = (cz * sy * sx) - (sz * cx)
        r02 = (cz * sy * cx) + (sz * sx)
        
        r10 = sz * cy
        r11 = (sz * sy * sx) + (cz * cx)
        r12 = (sz * sy * cx) - (cz * sx)
        
        r20 = -sy
        r21 = cy * sx
        r22 = cy * cx
        
        # Construct the rotation matrix
        rotation_matrix = np.array([
            [r00, r01, r02],
            [r10, r11, r12],
            [r20, r21, r22]
        ])
        
        return rotation_matrix


    def main(self):
        self.get_logger().info("Starting to save data")
        while rclpy.ok():
            if self.data_num < 500:
                self.stack_rpy_data(self.imu_to_rpy(self.imu_data), "stand")
                print("Stand")
            elif self.data_num < 800:
                print("Rest")
            elif self.data_num < 1300:
                self.stack_rpy_data(self.imu_to_rpy(self.imu_data), "stoop")
                print("Stoop")
            else:
                self.get_logger().info("Finished saving data")
                break
            self.data_num += 1
            rclpy.spin_once(self, timeout_sec=0.1)
        self.set_bodyfixed_mat()

def main(args=None):
    rclpy.init(args=None)
    try:
        saver = calc_bf_quat()
        saver.main()
        rclpy.spin(saver)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
        saver.destroy_node()


if __name__ == '__main__':
    main()


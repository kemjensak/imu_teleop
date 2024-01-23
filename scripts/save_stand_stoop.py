#!/usr/bin/env python3

import sys
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf_transformations import euler_from_quaternion
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from ament_index_python.packages import get_package_share_directory
import numpy as np
from math import degrees

stand_time = 5  # seconds
rest_time = 3  # seconds
stoop_time = 5  # seconds

class stand_stoop_saver(Node):
    def __init__(self):
        super().__init__('stand_stoop_saver')
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

    def set_param(self):
        stand_mean = np.mean(self.stand_data, axis=0)
        stoop_mean = np.mean(self.stoop_data, axis=0)

        yaml_file_path = os.path.join(get_package_share_directory('imu_teleop'), 'config', 'stand_stoop_mean.yaml')
        # yaml_file_path = "/home/kjs-dt/ros2_ws/colcon_ws/src/imu_teleop/config" + "/stand_stoop_mean.yaml"
        with open(yaml_file_path, 'w') as f:
            f.write("stand_mean: " + str(degrees(stand_mean[0])) + "," + str(degrees(stand_mean[1])) + "," + str(degrees(stand_mean[2])) + "\n")
            f.write("stoop_mean: " + str(degrees(stoop_mean[0])) + "," + str(degrees(stoop_mean[1])) + "," + str(degrees(stoop_mean[2])) + "\n")
            f.close()
        self.get_logger().info("Saved Stand mean to yaml: %s" % stand_mean)
        self.get_logger().info("Saved Stoop mean to yaml: %s" % stoop_mean)

        # Set parameters
        # self.declare_parameters(
        #     namespace='',
        #     parameters=[
        #         ('stand_mean', list(stand_mean)),
        #         ('stoop_mean', list(stoop_mean))
        #     ]
        # )
        # self.get_logger().info("Saved Stand mean to parameter: %s" % stand_mean)
        # self.get_logger().info("Saved Stoop mean to parameter: %s" % stoop_mean)

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
        self.set_param()

def main(args=None):
    rclpy.init(args=None)
    try:
        saver = stand_stoop_saver()
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


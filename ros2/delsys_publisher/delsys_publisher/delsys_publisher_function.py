import os
import pathlib
import time

import numpy as np

import rclpy
from rclpy.node import Node
from delsys_messages.msg import DelsysIMU


abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)

class DelsysPublisher(Node):

    def __init__(self):
        super().__init__('delsys_publisher')
        self.publisher_ = self.create_publisher(DelsysIMU, 'delsys_imu_values', 100)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    
    def timer_callback(self):
        ### Read the sensors data

        # Publish temperatures array
        imuData = DelsysIMU()
        imuData.emg = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        imuData.acc_x = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        imuData.acc_y = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        imuData.acc_z = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        imuData.gyro_x = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        imuData.gyro_y = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        imuData.gyro_z = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        ### Publish data
        self.get_logger().info(f'Publishing: {imuData}')
        self.publisher_.publish(imuData)


def main(args=None):
    rclpy.init(args=args)

    publisher = DelsysPublisher()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()

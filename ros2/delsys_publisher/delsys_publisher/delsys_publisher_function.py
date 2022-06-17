import os
import pathlib
import time

import numpy as np

import rclpy
from rclpy.node import Node
from delsys_messages.msg import DelsysIMU

try:
    import pytrigno
except ImportError:
    import sys
    sys.path.insert(0, '..')
    import pytrigno


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
        self.trigno = pytrigno.TrignoIMU(n_sensors = 6, host='172.31.1.73',
                 cmd_port=50040, emg_port=50043, data_port=50044, timeout=10)
        self.trigno.start()

    
    def timer_callback(self):
        ### Read the sensors data
        data = self.trigno.getData().squeeze()
        # Publish temperatures array
        imuData = DelsysIMU()
        imuData.emg = self.trigno.getEMG().squeeze()
        imuData.acc_x = data[[i for i in range(0, 36, 6)]]
        imuData.acc_y = data[[i for i in range(1, 36, 6)]]
        imuData.acc_z = data[[i for i in range(2, 36, 6)]]
        imuData.gyro_x = data[[i for i in range(3, 36, 6)]]
        imuData.gyro_y = data[[i for i in range(4, 36, 6)]]
        imuData.gyro_z = data[[i for i in range(5, 36, 6)]]

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

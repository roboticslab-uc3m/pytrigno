import os
import numpy as np

import rclpy
from rclpy.node import Node

from roboasset_msgs.msg import DelsysIMU
from roboasset_msgs.msg import DelsysEMG

from pytrigno.imu_reader import TrignoIMU

abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)

class DelsysPublisher(Node):

    def __init__(self, host_ip):
        super().__init__('delsys_publisher')
        self.publisher_imu_ = self.create_publisher(DelsysIMU, 'delsys_imu_values', 100)
        self.publisher_emg_ = self.create_publisher(DelsysEMG, 'delsys_emg_values', 100)

        self.get_logger().info(f'host_ip: {host_ip}')

        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.trigno = TrignoIMU(
            n_sensors = 16,
            host = host_ip,
            cmd_port=50040,
            emg_port=50043,
            data_port=50044,
            timeout=10
        )
        self.trigno.start()


    def publish_emg(self):
        ### Read the sensors data
        data = self.trigno.getEMG()
        data = np.mean(data, axis = 1)
        emgData = DelsysEMG()
        emgData.emg = data

        ### Publish data
        self.get_logger().info(f'Publishing: {emgData}')
        self.publisher_emg_.publish(emgData)

    def publish_imu(self):
        N_CHANNELS = 9 # Number of channels is 9 according to official documentationç
        N_SENSORS = self.trigno.total_sensors
        TOTAL_CHANNELS = N_CHANNELS * N_SENSORS
        data = self.trigno.getData().squeeze()
        imuData = DelsysIMU()

        imuData.acc_x = data[[i for i in range(0, TOTAL_CHANNELS, N_CHANNELS)]]
        imuData.acc_y = data[[i for i in range(1, TOTAL_CHANNELS, N_CHANNELS)]]
        imuData.acc_z = data[[i for i in range(2, TOTAL_CHANNELS, N_CHANNELS)]]
        imuData.gyro_x = data[[i for i in range(3, TOTAL_CHANNELS, N_CHANNELS)]]
        imuData.gyro_y = data[[i for i in range(4, TOTAL_CHANNELS, N_CHANNELS)]]
        imuData.gyro_z = data[[i for i in range(5, TOTAL_CHANNELS, N_CHANNELS)]]

        ### Publish data
        self.get_logger().info(f'Publishing: {imuData}')
        self.publisher_imu_.publish(imuData)


def main(args=None):
    rclpy.init(args=args)

    publisher = DelsysPublisher(os.getenv('HOST_IP', '127.0.0.1'))

    while rclpy.ok():
        publisher.publish_emg()
        publisher.publish_imu()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()

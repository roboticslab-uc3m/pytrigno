"""
Tests communication with and data acquisition from a Delsys Trigno wireless
EMG system. Delsys Trigno Control Utility needs to be installed and running,
and the device needs to be plugged in. Tests can be run with a device connected
to a remote machine if needed.

The tests run by this script are very simple and are by no means exhaustive. It
just sets different numbers of channels and ensures the data received is the
correct shape.

Use `-h` or `--help` for options.
"""

import argparse
import time

try:
    import pytrigno
except ImportError:
    import sys
    sys.path.insert(0, '..')
    import pytrigno
    
def check_imu(host):
    dev = pytrigno.TrignoIMU(n_sensors = 16,
                               host=host)

    dev.start()
    print('########### TEST IMU EMG DATA ########### ')
    for i in range(30):
        print(f'### EMG DATA {i} ###')
        data = dev.getEMG().squeeze()

        print('### EMG ###')
        print(data)
        print()

        print(f'### IMU DATA {i} ###')
        data = dev.getData().squeeze()
        print(data)
        print('### Acc X ###')
        print(data[[i for i in range(0, 144, 9)]])
        print()

        print('### Acc Y ###')
        print(data[[i for i in range(1, 144, 9)]])
        print()

        print('### Acc Z ###')
        print(data[[i for i in range(2, 144, 9)]])
        print()

        print('### Gyro X ###')
        print(data[[i for i in range(3, 144, 9)]])
        print()

        print('### Gyro Y ###')
        print(data[[i for i in range(4, 144, 9)]])

        print()
        print('### Gyro Z ###')
        print(data[[i for i in range(5, 144, 9)]])
        print()

    dev.stop()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument(
        '-a', '--addr',
        dest='host',
        default='localhost',
        help="IP address of the machine running TCU. Default is localhost.")
    args = parser.parse_args()
    check_imu(args.host)

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
    for i in range(5):
        print(f'### EMG DATA {i} ###')
        data = dev.getEMG()
        print(data)

        print(f'### IMU DATA {i} ###')
        data = dev.getData()
        print(data)
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

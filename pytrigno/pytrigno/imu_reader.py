from pytrigno.base_daq import BaseTrignoDaq


class TrignoIMU(BaseTrignoDaq):
    """
    Delsys Trigno wireless EMG system IMU data.

    Requires the Trigno Control Utility to be running.

    Parameters
    ----------
    n_sensors : Number of sensors
        Sensor to use, e.g. (lowchan, highchan) obtains data from
        channels lowchan through highchan. Each sensor has three accelerometer
        channels.
    host : str, optional
        IP address the TCU server is running on. By default, the device is
        assumed to be attached to the local machine.
    cmd_port : int, optional
        Port of TCU command messages.
    emg_port : int, optional
        Port of TCU IMU EMG data access. By default, 50043 is used, but it is
        configurable through the TCU graphical user interface.
        Number of seconds before socket returns a timeout exception.
    data_port : int, optional
        Port of TCU IMU data access. By default, 50044 is used, but
        it is configurable through the TCU graphical user interface.
    timeout : float, optional
        Number of seconds before socket returns a timeout exception.
    """
    def __init__(self, n_sensors, host='localhost',
                 cmd_port=50040, emg_port=50043, data_port=50044, timeout=10):

        super(TrignoIMU, self).__init__(
            host=host, cmd_port=cmd_port, emg_port=emg_port, data_port=data_port, total_sensors=n_sensors, timeout=timeout)

        units = 'V'

        self.scaler = 1.
        if units == 'mV':
            self.scaler = 1000.
        elif units == 'normalized':
            # max range of EMG data is 11 mV
            self.scaler = 1 / 0.011

    def getEMG(self, samples = 15):
        """
        Request a sample of EMG data from the device.

        This is a blocking method, meaning it returns only once the requested
        number of samples are available.

        Returns
        -------
        data : ndarray, shape=(n_sensors * n_channels, num_samples)
            Data read from the device. Each channel is a row and each column
            is a point in time.
        """
        data = super(TrignoIMU, self).read(samples = 1, is_emg = True, n_channels = 1)
        #data = data[self.channel_range[0]:self.channel_range[1]+1, :]
        return self.scaler * data

    def getData(self, samples = 1):
        """
        Request a sample of IMU data (acc and gyro) from the device.

        This is a blocking method, meaning it returns only once the requested
        number of samples are available.

        Returns
        -------
        data : ndarray, shape=(n_sensors * n_channels, num_samples)
            Data read from the device. Each channel is a row and each column
            is a point in time.
        """
        data = super(TrignoIMU, self).read(samples = 1, is_emg = False, n_channels = 9)
        #data = data[self.channel_range[0]:self.channel_range[1]+1, :]
        return data

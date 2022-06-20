import socket
import struct
import numpy

class _BaseTrignoDaq(object):
    """
    Delsys Trigno wireless EMG system.

    Requires the Trigno Control Utility to be running.

    Parameters
    ----------
    host : str
        IP address the TCU server is running on.
    cmd_port : int
        Port of TCU command messages.
    emg_port : int
        Port of TCU emg data access.
    data_port : int
        Second port of TCU data acces. Just needed if IMU is enabled
    emg_rate : int
        Sampling rate of the emg data source.
    rate: int
        Sampling rate of the second data source.
    total_sensors : int
        Total number of sensors supported by the device.
    timeout : float
        Number of seconds before socket returns a timeout exception

    Attributes
    ----------
    BYTES_PER_CHANNEL : int
        Number of bytes per sample per channel. EMG and accelerometer data
    CMD_TERM : str
        Command string termination.

    Notes
    -----
    Implementation details can be found in the Delsys SDK reference:
    http://www.delsys.com/integration/sdk/
    """

    BYTES_PER_CHANNEL = 4
    CMD_TERM = '\r\n\r\n'

    def __init__(self, host, cmd_port, emg_port, data_port, total_sensors, timeout):
        self.host = host
        self.cmd_port = cmd_port
        self.emg_port = emg_port
        self.data_port = data_port
        self.total_sensors = total_sensors
        self.timeout = timeout

        self._min_recv_size = self.total_sensors * self.BYTES_PER_CHANNEL

        self._initialize()

    def _initialize(self):

        # create command socket and consume the servers initial response
        self._comm_socket = socket.create_connection(
            (self.host, self.cmd_port), self.timeout)
        self._comm_socket.recv(1024)

        # create the emg data socket
        self._emg_socket = socket.create_connection(
            (self.host, self.emg_port), self.timeout)

        # create the imu data socket
        self._data_socket = socket.create_connection(
            (self.host, self.data_port), self.timeout)

    def start(self):
        """
        Tell the device to begin streaming data.

        You should call ``read()`` soon after this, though the device typically
        takes about two seconds to send back the first batch of data.
        """
        self._send_cmd('START')

    def read(self, samples, is_emg = True, n_channels = 1):
        """
        Request a sample of data from the device.

        This is a blocking method, meaning it returns only once the requested
        number of samples are available.

        Parameters
        ----------
        num_samples : int
            Number of samples to read per channel.
        is_emg : boolean
            True if emg data is requested. False if imu data is requested.
        n_channels : int
            Number of channels per sensor. For example EMG is 1, but IMU is 6 (acc X, acc Y, acc Z, gyro X, gyro Y, gyro Z)
        Returns
        -------
        data : ndarray, shape=(number_sensors * n_channels, num_samples)
            Data read from the device. Each channel is a row and each column
            is a point in time.
        """
        if is_emg:
            data_socket = self._emg_socket
        else:
            data_socket = self._data_socket

        l_des = samples * self._min_recv_size * n_channels
        l = 0
        packet = bytes()
        while l < l_des:
            try:
                packet += data_socket.recv(l_des - l)
            except socket.timeout:
                l = len(packet)
                packet += b'\x00' * (l_des - l)
                raise IOError("Device disconnected.")
            l = len(packet)

        data = numpy.asarray(
            struct.unpack('>'+'f'*self.total_sensors * n_channels * samples, packet))
        data = numpy.transpose(data.reshape((-1, self.total_sensors * n_channels)))

        return data

    def stop(self):
        """Tell the device to stop streaming data."""
        self._send_cmd('STOP')

    def reset(self):
        """Restart the connection to the Trigno Control Utility server."""
        self._initialize()

    def __del__(self):
        try:
            self._comm_socket.close()
        except:
            pass

    def _send_cmd(self, command):
        self._comm_socket.send(self._cmd(command))
        resp = self._comm_socket.recv(128)
        self._validate(resp)

    @staticmethod
    def _cmd(command):
        return bytes("{}{}".format(command, _BaseTrignoDaq.CMD_TERM),
                     encoding='ascii')

    @staticmethod
    def _validate(response):
        s = str(response)
        if 'OK' not in s:
            print("warning: TrignoDaq command failed: {}".format(s))

class TrignoIMU(_BaseTrignoDaq):
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
        
        self.n_sensors = n_sensors

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

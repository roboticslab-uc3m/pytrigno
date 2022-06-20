pytrigno
========

``TrignoIMU`` provide access to data served by Trigno
Control Utility for the Delsys Trigno wireless EMG system. TCU is Windows-only,
but this class can be used to stream data from it on another machine. TCU works
by running a TCP/IP server, with EMG data from the sensors on one port,
IMU data on another, and commands/responses on yet another. These
ports are configurable in the TCU GUI. The TCU program must be running before
a ``TrignoIMU`` object is created.

EMG data is in volts (by default) with a range of
±0.011 V. This can be converted to millivolts or normalized by the max range to
get a range of ±11 mV or ±1 (unitless), respectively.

Accelerometer data is sampled at 148.1 Hz and is in g.

Gyro data is sampled at 148.1 Hz and is in º/s

It is necessary for the correct operation that the sensors are active and connected to a PC with the Trigno Control Unity software running. The sensors must be in EMG + IMU mode (the mode can be changed from the Trigno Control Unity software).  By default, EMG data is sent through port 50043, acceleration and orientation data through 50044, and commands through 50400.

Auxiliary data values are multiplexed so that consecutive samples from the same auxiliary channel appear every 144 data values. Note also that data from the same sensor are grouped together in blocks of nine data values; data values 0 through 8 are from Avanti Sensor 1, data values 9 through 17 are from Avanti Sensor 2, and so on.

More info about how data is send can be found in the official documentation https://www.delsys.com/downloads/USERSGUIDE/trigno/sdk.pdf


This repository has the necessary infrastructure to publish the data in ros2. EMG data are published under the topic 'delsys_emg_values', while acceleration and orientation (IMU) data are published under the topic 'delsys_img_values'.

To run it, it is necessary to have the sensors ready and to compile the docker image. To do this, on the root of the project, run:

```
docker build . -t delsys-test --no-cache
```
Once the image is builded, run the next command:
```
docker run delsys-test
```

Now, the data is been published 

Dependencies
------------

- `NumPy <http://www.numpy.org/>`_

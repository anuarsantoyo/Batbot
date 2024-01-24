import minimalmodbus
import time
import pandas as pd

#!python -m serial.tools.list_ports
#daq_port = '/dev/ttyUSB0'

#measurements = read_measurements_df_6axis(port=daq_port, duration=5)
#measurements['Time'] = 0
#calib = measurements.mean().iloc[:,0]

#measurements = read_measurements_df_6axis(port=daq_port, duration=10) - calib


def twos_complement(value, bits):
    """Compute the 2's complement of int value."""
    if value & (1 << (bits - 1)):
        value -= 1 << bits
    return value
def read_measurements_df_6axis(port='/dev/ttyUSB0', duration=10):
    '''
    This function reads the measurements from a 6-axis force/torque sensor connected via Modbus RTU.
    It collects data for a specified duration and returns a pandas DataFrame with the measurements.

    The DataFrame includes time-series data for force and torque measurements along and around
    the X, Y, and Z axes. If a calibration setting is applied, the function can also return calibrated data.

    :param port: str, optional
        The serial port to which the sensor is connected. Default is '/dev/ttyUSB0'.
    :param duration: int or float, optional
        The duration in seconds for which to collect data from the sensor. Default is 10 seconds.
    :param calibration: bool

    :return: pandas.DataFrame
        A DataFrame containing the time-series data for the 6-axis measurements. Each force measurement
        is in Newtons and each torque measurement is in Newton-meters, with appropriate scaling applied.

    The returned DataFrame has the following columns:
    - 'Time': The time points of the measurement in seconds.
    - 'Fxyz': The force measurement along the XYZ-axis in Newtons.
    - 'Mxyz': The torque measurement around the XYZ-axis in Newton-meters.
    '''

    # Setting up the Modbus RTU connection
    instrument = minimalmodbus.Instrument(port, 1)
    instrument.serial.baudrate = 115200
    instrument.serial.parity = minimalmodbus.serial.PARITY_NONE
    instrument.mode = minimalmodbus.MODE_RTU

    # Store measurements for each of the 6 sensors
    measurements = [[] for _ in range(6)]
    time_points = []

    end_time = time.time() + duration  # Measure for duration time

    while time.time() < end_time:
        # Reading 12 registers starting from 2560 (0x0A00 in hex), which equals 24 bytes
        response = instrument.read_registers(2560, 12, functioncode=3)

        # Extracting the 6 quantities from the response and convert using two's complement
        quantities = [twos_complement((response[i] << 16) | response[i + 1], 32) for i in range(0, len(response), 2)]

        # Append measurements for each sensor
        for q_values, q in zip(measurements, quantities):
            q_values.append(q)

        time_points.append(time.time() - (end_time - duration))  # Record the measurement time

        time.sleep(0.003)  # Interval of 0.1 seconds

    # Convert data to a DataFrame for easy plotting with Plotly Express
    labels = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
    df = pd.DataFrame({'Time': time_points})
    for label, data in zip(labels, measurements):
        df[label] = data
    df['Mx'] = -df['Mx']
    df['Mz'] = -df['Mz']
    df[['Mx', 'My', 'Mz']] = df[['Mx', 'My', 'Mz']] / 10000
    df[['Fx', 'Fy', 'Fz']] = df[['Fx', 'Fy', 'Fz']] / 100

    return df
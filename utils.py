import matplotlib.pyplot as plt
import serial
import numpy as np
import urllib.request
from matplotlib.patches import Ellipse
import struct
from scipy.signal import savgol_filter, find_peaks
import minimalmodbus
import time
import pandas as pd
from scipy.interpolate import interp1d

sensors_col = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
calib = pd.read_csv('/home/anuarsantoyo/PycharmProjects/Batbot/analysis/forces/231115/data/df_calib.csv').mean()
calib['Time'] = 0


def command_batbot_V2(command, port):
    """
    This function takes a command and sends it to the Batbot.
    :param solution: list of commands [motor, attack_angle, neutral_state, amplitude]
    :param port: port of the wireless uart module.
    :return: None
    """
    motor, leg_x, leg_y, leg_x_amplitude, leg_y_amplitude, ellipse_angle = command
    motor = np.interp(motor, [0, 1], [260, 270])
    leg_x = np.interp(leg_x, [0, 1], [40, 120])
    leg_y = np.interp(leg_y, [0, 1], [30, 150])
    leg_x_amplitude = np.interp(leg_x_amplitude, [0, 1], [0, 40])
    leg_y_amplitude = np.interp(leg_y_amplitude, [0, 1], [0, 60])
    print(f"Sending command to batbot "
          f"leg_x: {leg_x}\n"
          f"leg_y: {leg_y}\n"
          f"leg_x_amplitude: {leg_x_amplitude}\n"
          f"leg_y_amplitude: {leg_y_amplitude}\n"
          f"Ellipse: {ellipse_angle}")
    ser = serial.Serial(port=port, baudrate=115200, bytesize=8, parity='N', stopbits=1)
    ser.write(str.encode(f'{motor},{leg_x},{leg_y},{leg_x_amplitude},{leg_y_amplitude},{ellipse_angle}'))
    print("Sent!\n")


def fitness_mse(measurements):
    """
    Calculates the fitness as the MSE, with the idea of trying to reduce to zero the sensors instead of just it's mean.
    The sensor 1 which has shown to have the largest values is just to slice the df from the first peak to the last
    valley.
    :param measurements: df of data provided by read_measurements_df()
    :return: score
    """
    y = measurements.sensor_1
    peaks, _ = find_peaks(y, height=0)
    df_sliced = measurements.iloc[peaks[0]: peaks[-1]]
    return (df_sliced.drop('timestamp', axis=1) ** 2).mean().sum()

def peak_slice_interpolate(measurements, args = {'peak_height':-400, 'peak_distance':10}):
    """
    Interpolates sensor data between the first and last detected peaks and c
    :param measurements:  (pandas.DataFrame) A DataFrame containing sensor data.
    :return: Interpolated data after slicing from first to last peak
    """
    df = measurements.copy()
    # Find peaks (adjust parameters as discussed previously)
    peaks, _ = find_peaks(df['Fz'], height=args['peak_height'], distance=args['peak_distance'])
    df_sliced = df.iloc[peaks[0]:peaks[-1] + 1].reset_index(drop=True)
    # Now create the new time series with 1000 points
    new_time = np.linspace(df_sliced['Time'].min(), df_sliced['Time'].max(), 1000)
    # Initialize a new DataFrame to store the interpolated values
    interpolated_df = pd.DataFrame(index=new_time)

    # Interpolate each sensor column
    for column in df_sliced.columns:
        if column != 'Time':
            # Create the interpolation function
            interp_func = interp1d(df_sliced['Time'], df_sliced[column], kind='linear', fill_value='extrapolate')
            # Apply the interpolation function to the new time series
            interpolated_df[column] = interp_func(new_time)

    # Reset the index to make 'Time' a column again
    interpolated_df.reset_index(inplace=True)
    interpolated_df.rename(columns={'index': 'Time'}, inplace=True)
    return interpolated_df, peaks

def fitness_avg_force(measurements, plot=False, args = {'peak_height':-400, 'peak_distance':10}):
    """
    Interpolates sensor data between the first and last detected peaks and calculates the mean
    of the interpolated 'Fy' and 'Fz' columns to compute a score as the Euclidean norm.

    :param measurements: (pandas.DataFrame) A DataFrame containing sensor data with at least 'Time',
                                       'Fy', and 'Fz' columns.

    :return: score: (float) The Euclidean norm of the mean values of 'Fy' and 'Fz' from the
                     interpolated data.
    """
    df = measurements.copy()
    interpolated_df, peaks = peak_slice_interpolate(df, args)

    # interpolated_df now contains 1000 interpolated data points based on the 'Time' column.

    Fy, Fz = interpolated_df.mean()[['Fy', 'Fz']]
    score = np.sqrt(Fy ** 2 + Fz ** 2)

    if plot:
        # Create the base line plot
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))
        # Plot 'Fz' column
        ax1.plot(df['Time'], df['Fz'], label='Fz')
        ax1.plot(df['Time'], df['Fy'], label='Fy', zorder=1)
        # Plot the peaks
        ax1.scatter(df['Time'][peaks], df['Fz'][peaks], color='red', s=10, label='Peaks', zorder=5)
        # Adding title and labels
        ax1.legend()
        ax1.set_title('Peaks found')
        ax1.set_xlabel('Time')
        ax1.set_ylabel('Force')
        # Setting up a simple plot with equal scaling on the axes
        # Setting the aspect of the plot to be equal.
        ax2.set_aspect('equal', adjustable='box')

        # Drawing a simple line for demonstration
        ax2.plot(interpolated_df.Fy, interpolated_df.Fz, zorder=1)
        ax2.scatter(Fy, Fz, c='r', marker='*')
        ax2.scatter(0, 0, c='g', marker='+')
        ax2.arrow(0, 0, Fy, Fz, head_width=40, head_length=40)

        # Setting labels for the axes
        ax2.set_xlabel('Fy')
        ax2.set_ylabel('Fz')
        ax2.set_title(f'Score: {round(score, 2)}')

        # Display the plot
        plt.show()

    return score


def twos_complement(value, bits):
    """Compute the 2's complement of int value."""
    if value & (1 << (bits - 1)):
        value -= 1 << bits
    return value


def read_measurements_df_6axis(port='/dev/ttyUSB0', duration=10, calibration=False):
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

    if calibration:
        return df-calib
    else:
        return df




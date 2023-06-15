import matplotlib.pyplot as plt
import serial
import numpy as np
import urllib.request
from matplotlib.patches import Ellipse
import time
import struct
import pandas as pd


def command_prototype(solution):
    """
    This function takes a proposed solution from the CMA optimization and commands the robot run it.
    :param solution: TBD
    :return: None
    """
    amplitude, frequency = solution
    amplitude = 90 * amplitude  # denormalization
    frequency = 5 * frequency  # denormalization
    print(f"Sending command to batbot amp: {amplitude}, freq: {frequency}...")
    html = urllib.request.urlopen(f"http://192.168.43.246/${amplitude},{frequency}$")
    print("Sent!")

def command_batbot2dof(solution, ip_address):
    """
    This function takes a proposed solution from the CMA optimization and commands the robot run it.
    :param solution: TBD
    :return: None
    """
    motor_speed, amplitude = solution
    amplitude = 9*amplitude + 1  # TODO: adapt to batbot2dof
    #motor_speed = 30 * motor_speed + 120 # denormalization
    motor_speed = (300)*motor_speed + 950
    print(f"Sending command to batbot motor speed: {motor_speed}, amplitude: {amplitude}...")
    try:
        html = urllib.request.urlopen(f"http://{ip_address}/${motor_speed},{amplitude}!", timeout=1)
    except TimeoutError:
        pass
    print("Sent!")


def fitness_prototype(measurements, plot=False):
    """
    Calculates fitness score of solution from the measured data
    :param measurements: np.aray of data provided by get_measurements()
    :return: score
    """
    time, values = measurements
    soll = lambda x: -0.68 * np.sin(0.00255 * x)
    goal = soll(time)
    error = goal - values
    if plot == True:
        plt.title("Best performing test from this generation")
        plt.plot(goal, label='Goal')
        plt.plot(values, label='measurements')
        plt.legend()
        plt.xlabel("time")
        plt.ylabel("a*sin(b*time)")
        plt.show()

    return np.square(error).mean()

def fitness_batbot2dof(measurements, plot=False):
    """
    Calculates fitness score of solution from the measured data
    :param measurements: np.aray of data provided by read_measurements_df()
    :return: score
    """
    if plot == True:
        measurements.drop('timestamp', axis=1).plot()
        plt.show()

    return (measurements.drop('timestamp', axis=1)**2).mean().sum()



def min_max_creator(min_value=210, max_value=970):  # Values experimentally calculated
    return lambda x: (2 / (max_value - min_value)) * x + 1 - 2 * max_value / (max_value - min_value)


def get_measurements(ser):
    """
    This function connects to the arduino and commands it to measure the (in arduino specified) input sensors for a
    determined amount of time. The data is then passed through serial communication and is further processed.
    :return: np.array with the shape nxm, where n is amount of data and m parameters (being the first one a timestamp).
    """

    ser.write('t'.encode())  # Send command to arduino to start measuring
    print("Measuring...")
    getData = ser.readline()  # Read serial data returned by arduino
    print('Done!')
    dataString = getData.decode('utf-8')  # Decode data
    array = np.fromstring(dataString[:-3], dtype=float, sep=',').reshape((-1, 2))  # Convert data into array

    array[:, 1] = np.interp(array[:, 1], (210, 970), (-1, +1))
    # linear equation to convert into -1,1 range
    # array[:, 1] = min_max(array[:, 1])

    array = np.unique(array, axis=0)  # Remove duplicates, arduino sometimes gets several data in the same millisecond
    return [array[:, 0], array[:, 1]]


def command_get_measurements(x):
    """
    This function connects to the arduino and commands to move and to measure the (in arduino specified) input sensors
    for a determined amount of time. The data is then passed through serial communication and is further processed.
    This is used mostly to test using only one arduino.
    :return: np.array with the shape nxm, where n is amount of data and m parameters (being the first one a timestamp).
    """
    arduino_port = "/dev/ttyACM0"  # serial port of Arduino
    baud = 9600  # arduino uno runs at 9600 baud
    ser = serial.Serial(arduino_port, baud)
    print("Connected to Arduino port:" + arduino_port)

    print("Reading sensor...")
    ser.write('t'.encode())
    getData = ser.readline()
    print("Read!")
    dataString = getData.decode('utf-8')
    array = np.fromstring(dataString[:-3], dtype=float, sep=',').reshape((-1, 2))
    return np.unique(array, axis=0)  # Remove duplicates, arduino sometimes gets several data in the same millisecond


def get_cov_ellipse(cov, centre, nstd, **kwargs):
    """
    Return a matplotlib Ellipse patch representing the covariance matrix
    cov centred at centre and scaled by the factor nstd.

    """

    # Find and sort eigenvalues and eigenvectors into descending order
    eigvals, eigvecs = np.linalg.eigh(cov)
    order = eigvals.argsort()[::-1]
    eigvals, eigvecs = eigvals[order], eigvecs[:, order]

    # The anti-clockwise angle to rotate our ellipse by
    vx, vy = eigvecs[:, 0][0], eigvecs[:, 0][1]
    theta = np.arctan2(vy, vx)

    # Width and height of ellipse to draw
    width, height = 2 * nstd * np.sqrt(eigvals)
    return Ellipse(xy=centre, width=width, height=height,
                   angle=np.degrees(theta), **kwargs)


def read_measurements_raw(port='/dev/ttyUSB0', duration=10):
    """
    Send command to BSQ-JN-P8 (DAQ) to return float measurements of all the sensors connected to it, and give back a list
    with the raw byte message with a timestamp of the measurement.

    :param port: path to the port the DAQ is connected to, which can be found with python -m serial.tools.list_ports
    :param duration: Duration of the measurements in seconds
    :return: A list of tuples in the form of [(timestamp 1, measurement 1), ...]
    """
    # Initialize serial connection. Parameters found from DAQ documentation.
    ser = serial.Serial(port=port, baudrate=115200, bytesize=8, parity='N', stopbits=1)
    measurements = []
    start = time.time()
    while time.time() - start < duration:  # Get measurements as long as the elapse time is smaller than the duration
        ser.write(b'\x01\x03\x00\xc8\x00\x10\xc5\xf8')  # Send command to DAQ to get floats of all channels
        # (from DAQ doc)
        measurements.append((time.time(), ser.read(37)))  # Create list of measurements.
        # I specified 37 as those are the bytes expected as my response, this allows me to have a higher sample
        # frequency as always only 37 bytes are read instead of readings getting mixed.
    ser.close()
    return measurements[1:]  # Omit first as it usually is a zero.


def bytes_to_float(h1, h2, h3, h4):
    """
    Convert bytes to float
    :param h1: first byte
    :param h2: second byte
    :param h3: third byte
    :param h4: fourth byte
    :return: float number of the given bytes
    """
    ba = bytearray()
    ba.append(h1)
    ba.append(h2)
    ba.append(h3)
    ba.append(h4)
    return struct.unpack("!f", ba)[0]


def measurements_to_df(measurement_bytes):
    """
    From list of measurements given by read_measurements_raw(), convert the values in float and return a data frame
    with the columns being: timestamp, sensor_1, ... , sensor_6
    :param measurement_bytes: measurements list from read_measurement_raw()
    :return: Dataframe with sensor readings.
    """
    dict_list = []  # Dictionaries list that will be converted to pandas Dataframe
    for ans in measurement_bytes:
        row = {'timestamp': ans[0]}  # Initialize row dictionary
        measure = ans[1]  # DAQ response at given timestamp
        for i in range(6):  # Iterate through the first 6 sensors (the ones being used)
            pos = 3 + 4 * i  # Initial index of the bytes (from 4 bytes) of the ith sensor.
            # Convert the 4 bytes from the sensor reading to float and save them in row dict.
            row[f'sensor_{i + 1}'] = bytes_to_float(measure[pos], measure[pos + 1], measure[pos + 2], measure[pos + 3])
        dict_list.append(row)  # Append row dict to list
    return pd.DataFrame(dict_list)  # Convert from list to dataframe


def read_measurements_df(port='/dev/ttyUSB0', duration=10, calibration=False):
    """
    Obtain measurements of the DAQ for a given duration and process them into a dataframe.
    :param port: path to the port the DAQ is connected to, which can be found with python -m serial.tools.list_ports
    :param duration: Duration of the measurements in seconds
    :param calibration: Boolean stating if a calibration step should be taken, or data should be returned uncalibrated.
    :return: Dataframe with sensor readings.
    """
    if calibration:  # TODO: Must recheck calibration idea, as when Batbot is mounted it already has a force that
        # can't be calibrated to zero.
        shift = measurements_to_df(read_measurements_raw(port=port, duration=1)).drop('timestamp', axis=1).mean()
        shift['timestamp'] = 0.0
    else:
        shift = 0
    df = measurements_to_df(read_measurements_raw(port=port, duration=duration))
    return df - shift

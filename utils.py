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
calib = pd.read_csv('/home/anuarsantoyo/PycharmProjects/Batbot/analysis/forces/data/df_calib3.csv').mean()
calib['Time'] = 0
#sensors_col = [f'sensor_{i}' for i in range(1, 7)]
#calibration = pd.read_csv('/analysis/sensor_calibration_BSQJNP8.csv')


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
    :param solution: motor_speed and amplitude (in values from [0,1])
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


def command_batbotV1(solution, port):
    """
    This function takes a proposed solution from the CMA optimization and commands the robot run it.
    :param solution: list of proposed solutions [motor, attack_angle, neutral_state, amplitude]
    :param port: port of the wireless uart module.
    :return: None
    """
    motor, attack_angle, neutral_state, amplitude = solution
    motor = np.interp(motor, [0, 1], [260, 280])
    attack_angle = np.interp(attack_angle, [0, 1], [90, 130])
    neutral_state = np.interp(neutral_state, [0, 1], [70, 170])
    amplitude = np.interp(amplitude, [0, 1], [0, 50])
    print(f"Sending command to batbot "
          f"motor: {motor}, "
          f"attack angel: {attack_angle}, "
          f"neutral state: {neutral_state}, "
          f"amplitude: {amplitude}...")
    ser = serial.Serial(port=port, baudrate=115200, bytesize=8, parity='N', stopbits=1)
    ser.write(str.encode(f'{motor},{attack_angle},{neutral_state},{amplitude}'))
    print("Sent!")


def command_batbotV2_2D(solution, port):
    """
    This function takes a proposed solution from the CMA optimization and commands the robot run it.
    :param solution: list of proposed solutions [motor, attack_angle, neutral_state, amplitude]
    :param port: port of the wireless uart module.
    :return: None
    """
    motor, attack_angle = solution
    motor = np.interp(motor, [0, 1], [260, 275])
    attack_angle = np.interp(attack_angle, [0, 1], [80, 120])
    leg_x = 50
    leg_y = 90
    leg_x_amplitude = 0
    leg_y_amplitude = 0
    ellipse_angle = 0
    print(f"Sending command to batbot "
          f"motor: {motor}, attack_angle: {attack_angle}")
    ser = serial.Serial(port=port, baudrate=115200, bytesize=8, parity='N', stopbits=1)
    ser.write(str.encode(f'{motor},{attack_angle},{leg_x},{leg_y},{leg_x_amplitude},{leg_y_amplitude},{ellipse_angle}'))
    print("Sent!")


def command_batbotV2_5D(solution, port):
    """
    This function takes a proposed solution from the CMA optimization and commands the robot run it.
    :param solution: list of proposed solutions [motor, attack_angle, neutral_state, amplitude]
    :param port: port of the wireless uart module.
    :return: None
    """
    motor, leg_x, leg_y, leg_x_amplitude, leg_y_amplitude, ellipse_angle = solution
    motor = np.interp(motor, [0, 1], [260, 275])
    leg_x = np.interp(leg_x, [0, 1], [50, 140])
    leg_y = np.interp(leg_y, [0, 1], [0, 180])
    leg_x_amplitude = np.interp(leg_x_amplitude, [0, 1], [0, 45])
    leg_y_amplitude = np.interp(leg_y_amplitude, [0, 1], [0, 90])
    print(f"Sending command to batbot "
          f"leg_x: {leg_x}\n"
          f"leg_y: {leg_y}\n"
          f"leg_x_amplitude: {leg_x_amplitude}\n"
          f"leg_y_amplitude: {leg_y_amplitude}\n"
          f"Ellipse: {ellipse_angle}")
    ser = serial.Serial(port=port, baudrate=115200, bytesize=8, parity='N', stopbits=1)
    ser.write(str.encode(f'{motor},{leg_x},{leg_y},{leg_x_amplitude},{leg_y_amplitude},{ellipse_angle}'))
    print("Sent!\n")

def command_batbotV2_allD(solution, port):
    """
    This function takes a proposed solution from the CMA optimization and commands the robot run it.
    :param solution: list of proposed solutions [motor, attack_angle, neutral_state, amplitude]
    :param port: port of the wireless uart module.
    :return: None
    """
    motor, attack_angle, leg_x, leg_y, leg_x_amplitude, leg_y_amplitude, ellipse_angle = solution
    motor = np.interp(motor, [0, 1], [260, 300])
    attack_angle = np.interp(attack_angle, [0, 1], [80, 120])
    leg_x = np.interp(leg_x, [0, 1], [50, 140])
    leg_y = np.interp(leg_y, [0, 1], [0, 180])
    leg_x_amplitude = np.interp(leg_x_amplitude, [0, 1], [0, 45])
    leg_y_amplitude = np.interp(leg_y_amplitude, [0, 1], [0, 90])
    print(f"Sending command to batbot "
          f"leg_x: {leg_x}\n"
          f"leg_y: {leg_y}\n"
          f"leg_x_amplitude: {leg_x_amplitude}\n"
          f"leg_y_amplitude: {leg_y_amplitude}\n"
          f"Ellipse: {ellipse_angle}")
    ser = serial.Serial(port=port, baudrate=115200, bytesize=8, parity='N', stopbits=1)
    ser.write(str.encode(f'{motor},{attack_angle},{leg_x},{leg_y},{leg_x_amplitude},{leg_y_amplitude},{ellipse_angle}'))
    print("Sent!\n")


def command_batbotV1_wifi(solution, ip_address):
    """
    This function takes a proposed solution from the CMA optimization and commands the robot run it.
    :param solution: list of proposed solutions [motor, attack_angle, neutral_state, amplitude]
    :return: None
    """
    motor, attack_angle, neutral_state, amplitude = solution
    motor = np.interp(motor, [0, 1], [600, 800])
    attack_angle = np.interp(attack_angle, [0, 1], [1150, 1450])
    neutral_state = np.interp(neutral_state, [0, 1], [1213, 1637])
    amplitude = np.interp(amplitude, [0, 1], [0, 213])
    print(f"Sending command to batbot "
          f"motor: {motor}, "
          f"attack angel: {attack_angle}, "
          f"neutral state: {neutral_state}, "
          f"amplitude: {amplitude}...")

    try:
        html = urllib.request.urlopen(f"http://{ip_address}/${motor},"
                                      f"{attack_angle},"
                                      f"{neutral_state},"
                                      f"{amplitude}!", timeout=3)  # The timeout closes the session and gives the
        # measurements time to start measuring after the command start
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


def fitness_batbotV1(measurements, plot=False, smooth=False):
    """
    Calculates fitness score of solution from the measured data
    :param measurements: np.aray of data provided by read_measurements_df()
    :param smooth: Boolean station if smoothing using savgol_filter is applied.
    :return: score
    """
    measurements = measurements.copy()
    if smooth:
        y = measurements[sensors_col]
        measurements[sensors_col] = savgol_filter(y, 10, 3, axis=0)
    if plot:
        measurements.plot(x='timestamp')
        plt.show()
    return (measurements.drop('timestamp', axis=1).mean()**2).sum()

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
    valleys, _ = find_peaks(-y, height=0)
    df_sliced = measurements.iloc[peaks[0]: valleys[-1]]
    return (df_sliced.drop('timestamp', axis=1) ** 2).mean().sum()

def fitness_mean(measurements):
    """
    Calculates the fitness as the MSE, with the idea of trying to reduce to zero the sensors instead of just it's mean.
    The sensor 1 which has shown to have the largest values is just to slice the df from the first peak to the last
    valley.
    :param measurements: df of data provided by read_measurements_df()
    :return: score
    """
    y = measurements.sensor_1
    peaks, _ = find_peaks(y, height=0)
    valleys, _ = find_peaks(-y, height=0)
    df_sliced = measurements.iloc[peaks[0]: valleys[-1]]
    return (df_sliced.drop('timestamp', axis=1).mean()**2).sum()

def fitness_avg_force(measurements, plot=False):
    """
    Interpolates sensor data between the first and last detected peaks and calculates the mean
    of the interpolated 'Fy' and 'Fz' columns to compute a score as the Euclidean norm.

    :param measurements: (pandas.DataFrame) A DataFrame containing sensor data with at least 'Time',
                                       'Fy', and 'Fz' columns.

    :return: score: (float) The Euclidean norm of the mean values of 'Fy' and 'Fz' from the
                     interpolated data.
    """

    df = measurements.copy()
    # Find peaks (adjust parameters as discussed previously)
    peaks, _ = find_peaks(df['Fz'], height=-400, distance=10)
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
        ax2.plot(df_sliced.Fy, df_sliced.Fz, zorder=1)
        ax2.scatter(Fy, Fz, c='r', marker='*')
        ax2.scatter(0, 0, c='g', marker='+')
        ax2.arrow(0, 0, Fy, Fz, head_width=40, head_length=40)

        # Setting labels for the axes
        ax2.set_xlabel('Fy')
        ax2.set_ylabel('Fz')
        ax2.set_xlim(-500, 500)
        ax2.set_ylim(-1000, 500)
        ax2.set_title(f'Score: {round(score, 2)}')

        # Display the plot
        plt.show()

    return score



def fitness_project(measurements, plot=False):
    """
    Calculates the fitness of a measurement slicing it from the back to learn the influence of the tail values and fits
     a linear function to project the real fitness were the tail values are no influence
    :param measurements: df of data provided by read_measurements_df()
    :return: score
    """
    scores_avg = []
    for i in range(1, 200):  # for a 5 seconds test we obtain 320 data, doing the analysis with the las 200 showed to
        # the most stable
        scores_avg.append(fitness_batbotV1(measurements[:-i], smooth=False))
    x = np.arange(len(scores_avg))
    fit = np.poly1d(np.polyfit(x, scores_avg, 1))
    if plot:
        plt.plot(x, scores_avg)
        plt.plot(x, fit(x))
        plt.show()
        measurements.plot(x='timestamp')
        plt.show()
    return fit(0)


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
        ser.write(b'\x01\x03\x00\xc8\x00\x10\xc5\xf8')  # Send command to DAQ to get floats of all channels (float)
        #ser.write(b'\x01\x03\x01\xf4\x00\x10\x04\x08')  # Send command to DAQ to get int of all channels (long int)

        # (from DAQ doc)
        measurements.append((time.time(), ser.read(37)))  # Create list of measurements.
        # I specified 37 as those are the bytes expected as my response, this allows me to have a higher sample
        # frequency as always only 37 bytes are read instead of readings getting mixed.
    ser.close()
    return measurements[1:]  # Omit first as it usually is a zero.

def read_measurements_6axis_raw(port='/dev/ttyUSB0', duration=10):
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
        ser.write(b'\x01\x03\x00\xc8\x00\x10\xc5\xf8')  # Send command to DAQ to get floats of all channels (float)
        #ser.write(b'\x01\x03\x01\xf4\x00\x10\x04\x08')  # Send command to DAQ to get int of all channels (long int)

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


def bytes_to_int(h1, h2, h3, h4):
    """
    Convert bytes to float
    :param h1: first byte
    :param h2: second byte
    :param h3: third byte
    :param h4: fourth byte
    :return: integer number of the given bytes
    """
    ba = bytearray()
    ba.append(h1)
    ba.append(h2)
    ba.append(h3)
    ba.append(h4)
    return struct.unpack("!i", ba)[0]


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

def twos_complement(value, bits):
    """Compute the 2's complement of int value."""
    if value & (1 << (bits - 1)):
        value -= 1 << bits
    return value

def read_measurements_df_6axis(port='/dev/ttyUSB0', duration=10, calibration=False):
    '''

    :param port:
    :param duration:
    :param calibration:
    :return:
    '''
    # Setting up the Modbus RTU connection
    instrument = minimalmodbus.Instrument(port, 1)
    instrument.serial.baudrate = 115200
    instrument.serial.parity = minimalmodbus.serial.PARITY_NONE
    instrument.mode = minimalmodbus.MODE_RTU

    # Store measurements for each of the 6 sensors
    measurements = [[] for _ in range(6)]
    time_points = []

    end_time = time.time() + duration  # Measure for 5 seconds

    while time.time() < end_time:
        # Reading 12 registers starting from 2560 (0x0A00 in hex), which equals 24 bytes
        response = instrument.read_registers(2560, 12, functioncode=3)

        # Extracting the 6 quantities from the response and convert using two's complement
        quantities = [twos_complement((response[i] << 16) | response[i + 1], 32) for i in range(0, len(response), 2)]

        # Append measurements for each sensor
        for q_values, q in zip(measurements, quantities):
            q_values.append(q)

        time_points.append(time.time() - (end_time - duration))  # Record the measurement time

        time.sleep(0.01)  # Interval of 0.1 seconds

    # Convert data to a DataFrame for easy plotting with Plotly Express
    labels = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
    df = pd.DataFrame({'Time': time_points})
    for label, data in zip(labels, measurements):
        df[label] = data
    df['Mx'] = -df['Mx']
    df['Mz'] = -df['Mz']
    if calibration:
        return df-calib
    else:
        return df

def read_measurements_df_BSQJNP8(port='/dev/ttyUSB0', duration=10, calibration=False):
    """
    Obtain measurements of the DAQ for a given duration and process them into a dataframe.
    :param port: path to the port the DAQ is connected to, which can be found with python -m serial.tools.list_ports
    :param duration: Duration of the measurements in seconds
    :param calibration: Boolean stating if a calibration step should be taken, or data should be returned uncalibrated.
    :return: Dataframe with sensor readings.
    """
    print('Reading measurements...')
    if calibration:
        calibration_data = get_sensor_calibration()
        shift = pd.Series(data=calibration_data['data'].to_list(), index=calibration_data['index'].to_list())
        shift['timestamp'] = 0.0
    else:
        shift = 0
    df = measurements_to_df(read_measurements_raw(port=port, duration=duration))
    df['timestamp'] = df['timestamp'] - df['timestamp'][0]
    print('Read!')
    return df - shift


def get_sensor_calibration():
    return pd.read_csv('/home/anuarsantoyo/PycharmProjects/Batbot/analysis/sensor_calibration_BSQJNP8.csv')

def get_one_data(port_obj):
    port_obj.reset_input_buffer()
    port_obj.write('r'.encode('ascii'))
    buf = port_obj.read(72)
    ret = [float(buf[3:12]),
           float(buf[15:24]),
           float(buf[27:36]),
           float(buf[39:48]),
           float(buf[51:60]),
           float(buf[63:72])]
    return ret


def read_measurements_df1(port, duration=10, sample_interval_ns=20_000_000):
    uart = serial.Serial(port, 115200, timeout=1)
    t_0 = time.time_ns()
    ret_df = pd.DataFrame(columns=['sensor_1',
                                   'sensor_2',
                                   'sensor_3',
                                   'sensor_4',
                                   'sensor_5',
                                   'sensor_6',
                                   'timestamp'])
    for i in range(1, int(duration*1_000_000_000/sample_interval_ns)+1):
        while time.time_ns() < t_0+i*sample_interval_ns:pass
        line = get_one_data(uart)
        line.append(time.time())
        ret_df.loc[i-1] = line
    return ret_df


def read_measurements_df(port, duration=10):
    uart = serial.Serial(port, 115200, timeout=1)
    t_0 = time.time()
    data = []
    while time.time() - t_0 < duration:
        data.append((time.time(), get_one_data(uart)))
        time.sleep(0.01)  # 0.001

    df_list = [{'timestamp': time-data[0][0],
                'sensor_1':sensors[0] - calibration.loc['sensor_1'][0],
                'sensor_2':sensors[1] - calibration.loc['sensor_2'][0],
                'sensor_3':sensors[2] - calibration.loc['sensor_3'][0],
                'sensor_4':sensors[3] - calibration.loc['sensor_4'][0],
                'sensor_5':sensors[4] - calibration.loc['sensor_5'][0],
                'sensor_6':sensors[5] - calibration.loc['sensor_6'][0]} for time, sensors in data]
    return pd.DataFrame(df_list)


def arctan2_degrees(y, x):
    angle = np.arctan2(y, x)
    angle_deg = np.degrees(angle)
    if angle_deg < 0:
        angle_deg += 360
    return angle_deg


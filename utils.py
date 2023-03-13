import matplotlib.pyplot as plt
import serial
import numpy as np
import urllib.request
from matplotlib.patches import Ellipse


def command_robot(solution):
    """
    This function takes a proposed solution from the CMA optimization and commands the robot run it.
    :param solution: TBD
    :return: None
    """
    amplitude, frequency = solution
    amplitude = 90*amplitude  # denormalization
    frequency = 5*frequency  # denormalization
    print(f"Sending command to batbot amp: {amplitude}, freq: {frequency}...")
    html = urllib.request.urlopen(f"http://192.168.43.246/${amplitude},{frequency}$")
    print("Sent!")

def fitness(measurements, plot=False):
    """
    Calculates fitness score of solution from the measured data
    :param measurements: np.array of data provided by get_measurements()
    :return: score
    """
    time, values = measurements
    soll = lambda x: -0.68 * np.sin(0.00255 * x)
    goal = soll(time)
    error = goal - values
    if plot==True:
        plt.title("Best performing test from this generation")
        plt.plot(goal, label='Goal')
        plt.plot(values, label='measurements')
        plt.legend()
        plt.xlabel("time")
        plt.ylabel("a*sin(b*time)")
        plt.show()

    return np.square(error).mean()


def min_max_creator(min_value=210, max_value=970):  # Values experimentally calculated
    return lambda x: (2 / (max_value-min_value)) * x + 1 - 2 * max_value / (max_value-min_value)



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
    #array[:, 1] = min_max(array[:, 1])

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
    vx, vy = eigvecs[:,0][0], eigvecs[:,0][1]
    theta = np.arctan2(vy, vx)

    # Width and height of ellipse to draw
    width, height = 2 * nstd * np.sqrt(eigvals)
    return Ellipse(xy=centre, width=width, height=height,
                   angle=np.degrees(theta), **kwargs)





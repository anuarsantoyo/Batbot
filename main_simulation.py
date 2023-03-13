from matplotlib import pyplot as plt

from utils import *
import numpy as np
from cmaes import CMA

def command_get_mesasurements_sim(x):
    step_size = x[1]
    amplitude = x[0]
    measurements = []
    steps =[]
    for step in range(0, 8000, 15):
        steps.append(step)
        measurements.append(np.sin((np.pi / 180) * step * step_size) * amplitude)
    return [np.array(steps), np.array(measurements)]

def fitness(measurements):
    """
    Calculates fitness score of solution from the measured data
    :param measurements: np.array of data provided by get_measurements()
    :return: score
    """
    time, values = measurements

    soll = lambda x: 60 * np.sin(3*(np.pi / 180)*x)

    error = soll(time) - values
    plt.plot(values, label='measurments')
    plt.plot(soll(time), label='goal')
    plt.legend()
    plt.show()
    return np.square(error).mean()


if __name__ == "__main__":
    # Start the CMA optimizer
    optimizer = CMA(mean=np.array([0.5, 0.5]), sigma=1.3, population_size=10, bounds=np.array([[0, 1], [0, 1]]))
    # Loop for all generations
    for generation in range(50):
        print(optimizer._C, '\n')
        print(optimizer.mean)
        solutions = []
        for i in range(optimizer.population_size):
            x = optimizer.ask()
            value = fitness(command_get_mesasurements_sim(x))
            solutions.append((x, value))
            print(f"#{generation} {value} (x1={x[0]}, x2 = {x[1]})")
        optimizer.tell(solutions)



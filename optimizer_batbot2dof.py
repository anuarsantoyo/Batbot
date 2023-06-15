from utils import *
import numpy as np
from cmaes import CMA
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms


# Connection details
port = "/dev/ttyUSB0"  # Find port using !python -m serial.tools.list_ports
ip_address = '192.168.111.130'  # Ip address with arduino

# Start the CMA optimizer
pop_size = 5
optimizer = CMA(mean=np.array([0.5, 0.5]), sigma=0.5, population_size=pop_size, bounds=np.array([[0, 1], [0, 1]]))
# Loop for all generations
for generation in range(50):
    print("\n")
    print("*"*50)
    print(f"Generation {generation}")
    print("*"*50)
    print("\n")

    solutions = []
    best_idx = 0
    best_value = np.inf
    for i in range(optimizer.population_size):
        print(f"Test: {i+1}/{pop_size}")
        x = optimizer.ask()
        command_batbot2dof(x, ip_address)
        measurements = read_measurements_df(port=port, duration=10, calibration=False)
        value = fitness_batbot2dof(measurements)
        if value < best_value:
            best_value = value
            best_idx = i
            best_measurement = measurements
        solutions.append((x, value))
        print(f"Result: {value} \n")
        time.sleep(5)


    # Visualization
    fig, (ax1, ax2) = plt.subplots(2, 1)
    fig.suptitle(f"Generation Results: {generation}")
    best_measurement.drop('timestamp', axis=1).plot(ax=ax1)

    x = []
    y = []
    c = []
    for solution in solutions:
        x.append(solution[0][0])
        y.append(solution[0][1])
        c.append(solution[1])

    ax2.scatter(x, y, c=c, cmap='coolwarm')
    ax2.set(xlabel="Amplitude (pmv)", ylabel="Angular Velocity (dt)", xlim=(0, 1), ylim=(0, 1))

    mean_x, mean_y = optimizer._mean
    n_std = 0.5

    cov = optimizer._C

    pearson = cov[0, 1]/np.sqrt(cov[0, 0] * cov[1, 1])
    # Using a special case to obtain the eigenvalues of this
    # two-dimensional dataset.
    ell_radius_x = np.sqrt(1 + pearson)
    ell_radius_y = np.sqrt(1 - pearson)
    ellipse = Ellipse((0, 0), width=ell_radius_x * 2, height=ell_radius_y * 2, alpha=0.1)

    # Calculating the standard deviation of x from
    # the squareroot of the variance and multiplying
    # with the given number of standard deviations.
    scale_x = np.sqrt(cov[0, 0]) * n_std
    # calculating the standard deviation of y ...
    scale_y = np.sqrt(cov[1, 1]) * n_std

    transf = transforms.Affine2D() \
        .rotate_deg(45) \
        .scale(scale_x, scale_y) \
        .translate(mean_x, mean_y)

    ellipse.set_transform(transf + ax2.transData)
    ax2.add_patch(ellipse)
    ax2.set_xticks(np.linspace(0, 1, 10), [950, 980, 1010, 1040, 1070, 1100, 1130, 1160, 1190, 2050])
    ax2.set_yticks(np.linspace(0, 1, 10), [1, 2, 3, 4, 5, 6, 7, 8, 9, 10])

    plt.show()  # savefig(f"./tests/pres_img2/gen_{generation}.png", dpi=600)

    optimizer.tell(solutions)

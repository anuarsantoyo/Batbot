import matplotlib.pyplot as plt

from utils import *
import numpy as np
from cmaes import CMA
import pickle
import seaborn as sns
import plotly.express as px
from pandas.plotting import parallel_coordinates

# Connection details
daq_port = "/dev/ttyUSB0"  # Find port using !python -m serial.tools.list_ports
command_port = "/dev/ttyACM1"


# Start the CMA optimizer
pop_size = 10
n_generation = 20
save_directory = "experiments/optimizer_batbotV2_5D/231107/test1/"  # TODO: dir
load = False
if load:
    file = open(save_directory+'optimizers/optimizer_49.pickle', 'rb')
    loaded_file = pickle.load(file)
    optimizer = loaded_file['optimizer']
    generation_0 = loaded_file['last_generation'] + 1
    file.close()
    results = pd.read_csv(save_directory+'results.csv')
else:
    optimizer = CMA(mean=np.array([0.5 for i in range(6)]),  # TODO: dim
                    sigma=0.5,
                    population_size=pop_size,
                    bounds=np.array([[0, 1] for i in range(6)]))  # TODO: dim
    results = pd.DataFrame(columns=['Generation',
                                    'Id',
                                    'Score',
                                    'Motor',
                                    'Leg x',
                                    'Leg y',
                                    'Amplitude x',
                                    'Amplitude y',
                                    'Ellipse'])  # TODO: dim
    generation_0 = 0

# df to plot scores
scores_plot = []
df_dict_list = []


# Loop for all generations
for generation in range(generation_0, generation_0+n_generation):
    print("\n")
    print("*"*50)
    print(f"Generation {generation+1}/{generation_0+n_generation}")
    print("*"*50)
    print("\n")

    solutions = []
    for i in range(optimizer.population_size):
        print(f"Test: {i+1}/{pop_size}")
        x = optimizer.ask()
        command_batbotV2_5D(x, command_port)  # TODO: dim
        time.sleep(1)  # To allow the Batbot to reach the attack angle and flapping speed
        measurements = read_measurements_df_6axis(port=daq_port, duration=5, calibration=True)
        score = fitness_avg_force(measurements, plot=True)
        print(f"Result: {score}")
        print("-" * 10)
        print("\n")
        time.sleep(1)

        measurements.to_csv(save_directory + f"measurements/{generation}_{i}({score}).csv", index=False)
        solutions.append((x, score))

        motor, leg_x, leg_y, leg_x_amplitude, leg_y_amplitude, ellipse_angle = x  # TODO:dim
        df_dict_list.append({'Generation': generation,
                             'Id': i,
                             'Score': score,
                             'Motor': motor,
                             'Leg x': leg_x,
                             'Leg y': leg_y,
                             'Amplitude x': leg_x_amplitude,
                             'Amplitude y': leg_y_amplitude,
                             'Ellipse': ellipse_angle})

    optimizer.tell(solutions)
    df = pd.DataFrame(df_dict_list)
    results = pd.concat([results, df], ignore_index=True)
    results.to_csv(save_directory + "results.csv", index=False)
    with open(save_directory+f"optimizers/optimizer_{generation}.pickle", "wb") as file:
        pickle.dump({'optimizer': optimizer, 'last_generation': generation}, file)

    # Visualization TODO: dim
    variables = results.drop(['Generation', 'Id', 'Score'], axis=1).columns

    fig, (ax1, ax2) = plt.subplots(1, 2, gridspec_kw={'width_ratios': [1, 0.1]}, figsize=(10, 6))
    # Plot the covariance matrix on ax1
    sns.heatmap(optimizer._C, annot=True, cmap='coolwarm', cbar=True, square=True,
                xticklabels=variables, yticklabels=variables, ax=ax1)
    # Plot the mean values on ax2
    sns.heatmap(optimizer._mean[:, np.newaxis], annot=True, cmap='vlag', cbar=False, yticklabels=variables, ax=ax2)
    ax2.set_title('Mean')
    ax2.set_xlabel('')
    ax2.set_yticks([])  # Hide the y-axis ticks
    plt.tight_layout()
    plt.show()

    sns.lineplot(data=results, x="Generation", y="Score")
    plt.savefig(save_directory + "score.jpg")
    plt.show()

    plt.figure(figsize=(12, 6))
    parallel_coordinates(results.drop(['Generation', 'Id'], axis=1), class_column='Score', colormap='viridis', alpha=0.5)
    plt.legend().set_visible(False)
    plt.title('Parallel Coordinates Plot')
    plt.ylabel('Parameter Value')
    plt.xticks(rotation=45)
    plt.tight_layout()  # Ensures that all elements of the plot fit well
    plt.savefig(save_directory + "results.jpg")
    plt.show()
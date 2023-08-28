import pandas as pd
from utils import *
import numpy as np
from cmaes import CMA
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms
import pickle
import seaborn as sns
import itertools


# Connection details
daq_port = "/dev/ttyUSB0"  # Find port using !python -m serial.tools.list_ports
command_port = "/dev/ttyACM1"


save_directory = "experiments/optimizer_batbotV2_2D/data/230828/test1/"
results = pd.DataFrame(columns=['Generation', 'Id', 'Score', 'Motor', 'Attack']) #,'Neutral', 'Amplitude'])TODO:dim
generation_0 = 0

# df to plot scores
scores_plot = []
df_dict_list = []
n_generation = 10

# Loop for all generations
for generation in range(generation_0, generation_0+n_generation):
    print("\n")
    print("*"*50)
    print(f"Generation {generation+1}/{generation_0+n_generation}")
    print("*"*50)
    print("\n")

    solutions = []
    for x_0, x_1 in itertools.product([0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1], repeat=2):
        x = (x_0, x_1)
        print(f"Test: {x}")

        command_batbotV2_2D(x, command_port)  #TODO:dim

        time.sleep(2)  # To allow the Batbot to reach the attack angle and flapping speed
        measurements = read_measurements_df(port=daq_port, duration=5)
        score = fitness_project(measurements)
        measurements.to_csv(save_directory + f"{generation}_{i}({score}).csv", index=False)
        solutions.append((x, score))
        motor, attack_angle = x  # , neutral_state, amplitude TODO:dim
        df_dict_list.append({'Generation': generation,
                             'Id': np.nan,
                             'Score': score,
                             'Motor': motor,
                             'Attack': attack_angle})  # ,'Neutral': neutral_state,'Amplitude': amplitude}) TODO:dim
        print(f"Result: {score} \n")
        time.sleep(1)


    df = pd.DataFrame(df_dict_list)
    results = pd.concat([results, df], ignore_index=True)
    results.to_csv(save_directory + "results.csv", index=False)

    # Visualization TODO:dim
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    pcm = ax.scatter(results['Motor'], results['Attack'], results['Score'])
    ax.set_xlabel('Motor')
    ax.set_ylabel('Attack')
    ax.set_zlabel('Score')
    ax.set_title(f'Generation: {generation}')
    fig.colorbar(pcm, ax=ax)
    plt.show()

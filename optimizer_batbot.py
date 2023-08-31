import pandas as pd
from utils import *
import numpy as np
from cmaes import CMA
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms
import pickle
import seaborn as sns

# Connection details
daq_port = "/dev/ttyUSB0"  # Find port using !python -m serial.tools.list_ports
command_port = "/dev/ttyACM1"


# Start the CMA optimizer
pop_size = 10
n_generation = 50

save_directory = "experiments/optimizer_batbotV2_2D/data/230830/test1/"

load = False
if load:
    file = open(save_directory+'optimizer_12.pickle', 'rb')
    loaded_file = pickle.load(file)
    optimizer = loaded_file['optimizer']
    generation_0 = loaded_file['last_generation'] + 1
    file.close()
    results = pd.read_csv(save_directory+'results.csv')
else:
    optimizer = CMA(mean=np.array([0.5, 0.5]),  # , 0.5, 0.5]), TODO:dim
                    sigma=0.5,
                    population_size=pop_size,
                    bounds=np.array([[0, 1], [0, 1]]))  # , [0, 1], [0, 1]])) TODO:dim
    results = pd.DataFrame(columns=['Generation', 'Id', 'Score', 'Motor', 'Attack']) #,'Neutral', 'Amplitude'])TODO:dim
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
        command_batbotV2_2D(x, command_port)  #TODO:dim

        time.sleep(1)  # To allow the Batbot to reach the attack angle and flapping speed
        measurements = read_measurements_df_oldDAQ(port=daq_port, duration=5)
        score = fitness_project(measurements)
        measurements.to_csv(save_directory + f"measurements/{generation}_{i}({score}).csv", index=False)
        solutions.append((x, score))
        motor, attack_angle = x  # , neutral_state, amplitude TODO:dim
        df_dict_list.append({'Generation': generation,
                             'Id': i,
                             'Score': score,
                             'Motor': motor,
                             'Attack': attack_angle})  # ,'Neutral': neutral_state,'Amplitude': amplitude}) TODO:dim
        print(f"Result: {score} \n")
        time.sleep(1)

    optimizer.tell(solutions)

    df = pd.DataFrame(df_dict_list)
    results = pd.concat([results, df], ignore_index=True)
    results.to_csv(save_directory + "measurements/results.csv", index=False)
    with open(save_directory+f"optimizers/optimizer_{generation}.pickle", "wb") as file:
        pickle.dump({'optimizer': optimizer, 'last_generation': generation}, file)

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

    print(f'Generations {generation} Optimizers Covariance matrix is now:\n')
    print(optimizer._C, optimizer._mean)
    sns.lineplot(data=results, x="Generation", y="Score")
    plt.show()

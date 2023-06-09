import pandas as pd
from utils import *
import numpy as np
from cmaes import CMA
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms
import pickle
import seaborn as sns

# Connection details
port = "/dev/ttyUSB1"  # Find port using !python -m serial.tools.list_ports
ip_address = '192.168.164.130'

# Start the CMA optimizer
pop_size = 5
n_generation = 5

save_directory = "experiments/optimizer_batbotV1/data/230711/test3/"
'''
file = open(save_directory+'optimizer_9.pickle', 'rb')
loaded_file = pickle.load(file)
optimizer = loaded_file['optimizer']
generation_0 = loaded_file['last_generation'] + 1
file.close()
results = pd.read_csv(save_directory+'results.csv')
'''
optimizer = CMA(mean=np.array([0.5, 0.5, 0.5, 0.5]),
                sigma=0.5,
                population_size=pop_size,
                bounds=np.array([[0, 1], [0, 1], [0, 1], [0, 1]]))
results = pd.DataFrame(columns=['Generation', 'Id', 'Score', 'Motor', 'Neutral', 'Amplitude'])
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
        #x = [0.01832196, 0.42284382, 0.15978612]
        command_batbotV1(x, ip_address)
        measurements = read_measurements_df(port=port, duration=5, calibration=True)
        score = fitness_project(measurements)
        measurements.to_csv(save_directory + f"{generation}_{i}.csv", index=False)
        solutions.append((x, score))
        motor, attack_angle, neutral_state, amplitude = x
        df_dict_list.append({'Generation': generation,
                             'Id': i,
                             'Score': score,
                             'Motor': motor,
                             'Angle': attack_angle,
                             'Neutral': neutral_state,
                             'Amplitude': amplitude})
        print(f"Result: {score} \n")
        time.sleep(7)

    optimizer.tell(solutions)

    df = pd.DataFrame(df_dict_list)
    results = pd.concat([results, df], ignore_index=True)
    results.to_csv(save_directory + "results.csv", index=False)
    with open(save_directory+f"optimizer_{generation}.pickle", "wb") as file:
        pickle.dump({'optimizer': optimizer, 'last_generation': generation}, file)

    # Visualization
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    pcm = ax.scatter(results['Motor'], results['Neutral'], results['Amplitude'], c=results['Score'])
    ax.set_xlabel('Motor')
    ax.set_ylabel('Neutral State')
    ax.set_zlabel('Amplitude')
    ax.set_title(f'Generation: {generation}')
    fig.colorbar(pcm, ax=ax)
    plt.show()

    print(f'Generations {generation} Optimizers Covariance matrix is now:\n')
    print(optimizer._C, optimizer._mean)
    sns.lineplot(data=results, x="Generation", y="Score")
    plt.show()

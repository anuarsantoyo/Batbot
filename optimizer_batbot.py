from utils import *
import numpy as np
from cmaes import CMA
import pickle
import seaborn as sns
import plotly.express as px

# Connection details
daq_port = "/dev/ttyUSB0"  # Find port using !python -m serial.tools.list_ports
command_port = "/dev/ttyACM0"


# Start the CMA optimizer
pop_size = 10
n_generation = 20
save_directory = "experiments/optimizer_batbotV2_5D/230919/test1/"

load = True
if load:
    file = open(save_directory+'optimizers/optimizer_59.pickle', 'rb')
    loaded_file = pickle.load(file)
    optimizer = loaded_file['optimizer']
    generation_0 = loaded_file['last_generation'] + 1
    file.close()
    results = pd.read_csv(save_directory+'results.csv')
else:
    optimizer = CMA(mean=np.array([0.5, 0.5, 0.5, 0.5, 0.5]),  # TODO:dim
                    sigma=0.5,
                    population_size=pop_size,
                    bounds=np.array([[0, 1], [0, 1], [0, 1], [0, 1], [0, 1]]))  # TODO:dim
    results = pd.DataFrame(columns=['Generation', 'Id', 'Score', 'Leg x', 'Leg y', 'Amplitude x', 'Amplitude y', 'Ellipse'])  # TODO:dim
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
        score = 1
        while score >= 1:  # Repeat until valid score
            command_batbotV2_5D(x, command_port)  #TODO:dim
            time.sleep(1)  # To allow the Batbot to reach the attack angle and flapping speed
            measurements = read_measurements_df_BSQJNP8(port=daq_port, duration=5, calibration=True)
            score = fitness_mse(measurements)
            print(f"Result: {score} \n")

        measurements.to_csv(save_directory + f"measurements/{generation}_{i}({score}).csv", index=False)
        solutions.append((x, score))
        leg_x, leg_y, leg_x_amplitude, leg_y_amplitude, ellipse_angle = x  # TODO:dim
        df_dict_list.append({'Generation': generation,
                             'Id': i,
                             'Score': score,
                             'Leg x': leg_x,
                             'Leg y': leg_y,
                             'Amplitude x': leg_x_amplitude,
                             'Amplitude y': leg_y_amplitude,
                             'Ellipse': ellipse_angle})  # TODO:dim

        time.sleep(1)

    optimizer.tell(solutions)

    df = pd.DataFrame(df_dict_list)
    results = pd.concat([results, df], ignore_index=True)
    results.to_csv(save_directory + "results.csv", index=False)
    with open(save_directory+f"optimizers/optimizer_{generation}.pickle", "wb") as file:
        pickle.dump({'optimizer': optimizer, 'last_generation': generation}, file)

    # Visualization TODO:dim
    '''results_plot = results.copy()
    results_plot['Generation'] = results_plot['Generation'].astype(str)
    fig = px.scatter_3d(results_plot, x='Motor', y='Attack', z='Score', color='Generation', hover_data='Id')  # TODO:dim
    fig.write_html(save_directory + "results.html")'''

    print(f'Generations {generation} Optimizers Covariance matrix is now:\n')
    print(optimizer._C, optimizer._mean)
    sns.lineplot(data=results, x="Generation", y="Score")
    plt.show()

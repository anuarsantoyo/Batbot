import pyfirmata
import numpy as np
import time
import matplotlib.pyplot as plt
from tqdm import tqdm
import seaborn as sns
import pandas as pd



board = pyfirmata.Arduino('/dev/ttyACM0')

it = pyfirmata.util.Iterator(board)

it.start()

forces = []
analog_input = board.get_pin('a:0:i')
step = 0.1
total_time = 15
amount_of_steps = int(total_time/step)
start = time.time()
for i in range(amount_of_steps):
    forces.append(analog_input.read())
    time.sleep(step)


print(f'Expexted time: {amount_of_steps*steps}, Real time: {time.time()-start}')








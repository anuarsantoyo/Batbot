import serial
import pandas as pd
import time


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


def read_measurements_df(port, duration=10, sample_interval_ns=20_000_000):
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





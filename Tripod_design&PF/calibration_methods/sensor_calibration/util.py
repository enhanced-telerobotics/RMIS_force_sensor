import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from cal_data import calData
from Error import error

# load file
def load_data(file_path):
    return pd.read_csv(file_path, header=None, sep=r'\s+', skiprows=0, converters={i: lambda x: x.replace(',', '') for i in range(100)})

def process_data(path):
    data = []
    cal_data = []
    cal_mat = []
    
    for data_path in path:
        # load data from path
        loaded_data = load_data('test/' + data_path)
        data.append(loaded_data)
        # split data for calibration
        cal_param = calData(loaded_data)
        cal_data.append(cal_param)
        # get calibration matrices
        cal_mat.append(cal_param.perform_cal())
    
    return cal_data, cal_mat

def testCalibration(cal_data, cal_mat):
    test_cal = []
    
    test_cal.append(np.dot(cal_data[0].SensorData, cal_mat[1]))
    test_cal.append(np.dot(cal_data[1].SensorData, cal_mat[0]))

    return test_cal

# print errors
def print_error(caltest, data):
    err = error(data[1].ForceData, caltest[1])
    err.print_error()

# plot graphs
def plot_calibration(cal_data, test_cal, cal_mat):
    color = ['k', 'b', 'r', 'g']
    titles = ['x', 'y', 'z']
    fig, axes = plt.subplots(3, 1, figsize=(5, 5))
    for i in range(3):
        ax = axes[i]
        ax.plot(cal_data[1].Time, cal_data[1].ForceData[:, i], color=color[0], label='Actual Force', linewidth=1)
        ax.plot(cal_data[1].Time, test_cal[1][:, i], color=color[1], label='Measured Force', linewidth=1)
        ax.set_title(f'{titles[i]}')
        ax.set_xlabel('Time(s)')
        ax.set_ylabel('Force(N)')
        ax.legend()

    plt.tight_layout()
    plt.show()
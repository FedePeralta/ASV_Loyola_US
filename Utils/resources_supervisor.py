import psutil
import numpy as np
from time import time, sleep
import argparse


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('time', type=int, help="Time for calculation in seconds")

    time0 = time()
    args = parser.parse_args()
    max_time = args.time

    data = np.array([[psutil.cpu_percent(), psutil.virtual_memory().percen]])

    while time()-time0 < max_time:

        cpu = psutil.cpu_percent()
        mem = psutil.virtual_memory().percent
        
        data = np.vstack((data, [cpu, mem]))
        
    print('Measurement process ended!')
    
    np.savetxt('resultado_consumo_computacional.csv', data, delimiter=',')
    
    











"""
    Simple python script to plot the difference between values
"""

import matplotlib.pyplot as plt
import numpy as np
import sys

if __name__ == '__main__':

    with open(sys.argv[1], "r") as f:
        values = f.read()
        values = np.array([v.split(";") for v in values.split(",") if len(v) > 0])

        plt.title(sys.argv[1])

        plt.plot([x for x in range(len(values))], values[:,0], 'blue', label='True')
        plt.plot([x for x in range(len(values))], values[:,1], 'orange', label='Predicted')
        plt.legend()
        plt.show()

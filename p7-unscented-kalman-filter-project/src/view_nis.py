"""
    Simple python script to plot NIS values
"""

import matplotlib.pyplot as plt
import sys

if __name__ == '__main__':

    df = sys.argv[2];

    with open(sys.argv[1], "r") as f:
        nis_values = f.read()
        nis_values = [float(v) for v in nis_values.split(",") if len(v) > 0]

        out_percentage = 0;
        for value in nis_values:
            if value > 7.815:
                out_percentage += 1
        print("Percentage of above value:", out_percentage / len(nis_values))

        plt.plot([x for x in range(len(nis_values))], nis_values, "orange", label="NIS value")
        plt.plot([x for x in range(len(nis_values))], [7.815 for x in range(len(nis_values))], 'blue', label='NIS Threshold')
        plt.legend()
        plt.show()

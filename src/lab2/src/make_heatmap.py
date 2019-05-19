import numpy as np
import matplotlib 
matplotlib.use('Agg') 
import matplotlib.pyplot as plt
from matplotlib import rcParams
import csv
import time
import warnings

def makeHeatMap(data, names, color, outputFileName):
    #to catch "falling back to Agg" warning
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        #code source: http://stackoverflow.com/questions/14391959/heatmap-in-matplotlib-with-pcolor
        fig, ax = plt.subplots()
        #create the map w/ color bar legend
        heatmap = ax.pcolor(data, cmap=color)
        cbar = plt.colorbar(heatmap)
        
        cbar.set_label('Error')
        # put the major ticks at the middle of each cell
        ax.set_xticks(np.arange(data.shape[0])+0.5, minor=False)
        ax.set_yticks(np.arange(data.shape[1])+0.5, minor=False)
        for j, lab in enumerate(['Low','Average','High']):
            cbar.ax.text(.5, (2 * j + 1) / 6.0, lab, ha='center', va='center')
        # want a more natural, table-like display
        ax.invert_yaxis()
        ax.xaxis.tick_top()
        ax.set_ylabel('Group Name')
        ax.set_xlabel('Group ID')
        ax.set_xticklabels(range(1, 21)) #ax.set_xticklabels(range(1, 21))
        ax.set_yticklabels(names)
        ax.set_title('Cost Heat Map', pad=20)
        plt.tight_layout()
        plt.savefig(outputFileName +'.png', format = 'png')
        plt.close()



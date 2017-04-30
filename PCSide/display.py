import maparray
import matplotlib.pyplot as plt
import numpy as np


def display():
    """ Displays the map"""

    tempArray = np.copy(maparray.mapArray)
    tempArray[maparray.curX, maparray.curY] = 0.25
    plt.imshow(tempArray, vmin=0.0, vmax=1.0)
    plt.gray()
    plt.draw()
    plt.pause(0.0001)
    plt.savefig('map.png')
    return

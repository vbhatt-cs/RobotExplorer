import maparray
import matplotlib.pyplot as plt


def display():
    """ Displays the map"""

    plt.imshow(maparray.mapArray, vmin=0.0, vmax=1.0)
    plt.gray()
    plt.draw()
    plt.pause(0.0001)
    plt.savefig('map.png')
    return

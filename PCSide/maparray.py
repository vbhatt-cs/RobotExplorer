import numpy as np

MAP_SIZE = 5

""" Map:
    0 (black) => Wall
    0.5 (grey) => Unexplored
    1 (white) => Open"""
mapArray = np.ones((MAP_SIZE, MAP_SIZE))

explorationStack = []  # Stack for flood fill algorithm

# Current location of the bot
curX = 0
curY = 0


def init():
    """ Initializes the value of global variables"""
    global mapArray, explorationStack, curX, curY

    mapArray = np.ones((MAP_SIZE, MAP_SIZE)) * 0.5
    explorationStack = []
    curX = int(MAP_SIZE / 2)
    curY = int(MAP_SIZE / 2)

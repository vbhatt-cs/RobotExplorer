import maparray
import numpy as np


def init():
    """ Initializes the exploration stack and sets the initial position of the bot a open"""

    maparray.mapArray[maparray.curX, maparray.curY] = 1
    maparray.explorationStack = [(maparray.curX, maparray.curY)]


def explore():
    """ Checks the next cell to explore and gives the bot distance and 
        direction required to reach that cell
        Uses maparray.explorationStack for exploring
        Uses maparray.maparray.mapArray for checking explored areas
        Uses maparray.curX, maparray.curY for current location of the bot"""

    nextDis = 0
    nextDir = 0

    nextCell = unexpNeighbor(maparray.curX, maparray.curY)
    if maparray.explorationStack[-1] != (maparray.curX, maparray.curY):  # Will be false if there is a wall
        maparray.explorationStack.append((maparray.curX, maparray.curY))

    if nextCell:  # Neighbor found
        nextDis = maparray.CELL_DIS
        nextDir = findPath(nextCell)
    else:  # Backtrack
        maparray.explorationStack.pop()
        if maparray.explorationStack:
            nextDis = maparray.CELL_DIS
            nextDir = findPath(maparray.explorationStack[-1])

    return nextDis, nextDir


def finished():
    """ Returns whether everything that can be explored has been explored"""
    return not (maparray.explorationStack or np.count_nonzero(maparray.mapArray == 0.5))


def unexpNeighbor(px, py):
    """ Finds an unexplored neighbor. Returns null if all are explored"""

    if py > 0 and maparray.mapArray[px, py - 1] == 0.5:
        return px, py - 1
    elif py < maparray.MAP_SIZE - 1 and maparray.mapArray[px, py + 1] == 0.5:
        return px, py + 1
    elif px > 0 and maparray.mapArray[px - 1, py] == 0.5:
        return px - 1, py
    elif px < maparray.MAP_SIZE - 1 and maparray.mapArray[px + 1, py] == 0.5:
        return px + 1, py
    else:
        return ()


def findPath(nextCell):
    """ Finds the path to next cell from current position
        Since only one cell is moved at a time, it is enough to tell the distance"""

    dx = nextCell[0] - maparray.curX
    dy = nextCell[1] - maparray.curY

    if dx == 1:
        return 0
    elif dx == -1:
        return 180
    elif dy == 1:
        return 90
    elif dy == -1:
        return -90
    return

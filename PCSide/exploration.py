import maparray


def init():
    maparray.mapArray[maparray.curX, maparray.curY] = 1
    maparray.explorationStack = neighbors(maparray.curX, maparray.curY)


def explore():
    """ Checks the next cell to explore and gives the bot distance and 
        direction required to reach that cell
        Uses maparray.explorationStack for exploring
        Uses maparray.mapArray for checking explored areas
        Uses maparray.curX, maparray.curY for current location of the bot"""
    nextDis = 0
    nextDir = 0

    return nextDis, nextDir


def finished():
    """ Returns whether everything that can be explored has been explored"""
    return maparray.explorationStack


def neighbors(px, py):
    neighborList = []
    if py > 0:
        neighborList.append((px, py - 1))
    if py < maparray.MAP_SIZE - 1:
        neighborList.append((px, py + 1))
    if px > 0:
        neighborList.append((px - 1, py))
    if px < maparray.MAP_SIZE - 1:
        neighborList.append((px + 1, py))

    return neighborList

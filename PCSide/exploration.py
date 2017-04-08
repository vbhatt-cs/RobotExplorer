import maparray


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
    done = 0
    return done

import maparray

angleDelta = 10

def updateMap(yaw, enc, wall, nextDis, nextDir):
    """ Updates the map using the sensor data from arduino
        Kept as a separate function to add error correction if required later"""

    if nextDir - angleDelta <= yaw <= nextDir + angleDelta:
        if not wall and enc == nextDis:
            if nextDir == 0:
                maparray.curX = maparray.curX + 1
                maparray.mapArray[maparray.curX, maparray.curY] = 1
            elif nextDir == 90:
                maparray.curY = maparray.curY + 1
                maparray.mapArray[maparray.curX, maparray.curY] = 1
            elif nextDir == 180:
                maparray.curX = maparray.curX - 1
                maparray.mapArray[maparray.curX, maparray.curY] = 1
            else:
                maparray.curY = maparray.curY - 1
                maparray.mapArray[maparray.curX, maparray.curY] = 1
        elif enc == nextDis:
            if nextDir == 0:
                maparray.mapArray[maparray.curX + 1, maparray.curY] = 0
            elif nextDir == 90:
                maparray.mapArray[maparray.curX, maparray.curY + 1] = 0
            elif nextDir == 180:
                maparray.mapArray[maparray.curX - 1, maparray.curY] = 0
            else:
                maparray.mapArray[maparray.curX, maparray.curY - 1] = 0
        else:
            """ Bot didn't move fully"""

    else:
        """ Bot didn't move in correct angle"""
    return

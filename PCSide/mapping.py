import mapArray


def updateMap(yaw, enc, wall, nextDis, nextDir):
    """ Updates the map using the sensor data from arduino
        Kept as a separate function to add error correction if required later"""
        if(yaw==nextDir and enc==nextDis):
            if(wall!=1):
                if(nextDir==0):
                    curX = curX+1
                    mapArray[curX,curY] = 1
                elif(nextDir==90):
                    curY = curY+1
                    mapArray[curX,curY] = 1
                elif(nextDir==180):
                    curX = curX-1
                    mapArray[curX,curY] = 1
                else:
                    curY=curY-1
                    mapArray[curX,curY] = 1
            else:
                if(nextDir==0):
                    mapArray[curX+1,curY] = 0
                elif(nextDir==90):
                    mapArray[curX,curY+1] = 0
                elif(nextDir==180):
                    mapArray[curX-1,curY] = 0
                else:
                    mapArray[curX,curY-1] = 0
    "else: what to do if the bot moved wrongly
        
    return

def getSensors():
    """ Get sensor data from arduino
        yaw -> angle with bot x-axis (float)
        enc -> encoder count (int)
        wall -> whether a wall is detected in front (bool)
        reached -> whether the distance given has been covered by the bot"""
    yaw = 0
    enc = 0
    wall = 0
    reached = 0
    return yaw, enc, wall, reached


def sendData(distance, direction):
    """ Send the next distance and direction to arduino"""
    return

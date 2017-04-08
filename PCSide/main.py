import maparray
import communication
import mapping
import exploration
import display

maparray.init()

nextDis = 0
nextDir = 0

while not exploration.finished():
    yaw, enc, wall, reached = communication.getSensors()
    mapping.updatePath(yaw, enc, wall, nextDis, nextDir)
    if reached or wall:
        nextDis, nextDir = exploration.explore()
        communication.sendData(nextDis, nextDir)
    display.display()

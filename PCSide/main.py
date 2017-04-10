import maparray
import communication
import mapping
import exploration
import display

nextDis = 0
nextDir = 0

maparray.init()  # Initialize global variables
communication.init()
exploration.init()

while not exploration.finished():
    yaw, enc, wall, reached = communication.getSensors()  # Get sensor data
    mapping.updateMap(yaw, enc, wall, nextDis, nextDir)  # Update map
    if reached or wall:
        nextDis, nextDir = exploration.explore()  # Continue exploration
        communication.sendData(nextDis, nextDir)  # Send command to bot
    display.display()

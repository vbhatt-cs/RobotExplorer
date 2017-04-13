from socket import *
from parse import *

address = ('192.168.137.149', 4210)  # IP, port of arduino
client_socket = socket(AF_INET, SOCK_DGRAM)  # Set up the socket for UDP


def init():
    """ Initialize UDP connection """

    global client_socket

    client_socket = socket(AF_INET, SOCK_DGRAM)  # Set up the socket for UDP
    client_socket.settimeout(5)  # Timeout = 1s


def getSensors():
    """ Get sensor data from arduino
        yaw -> angle with bot x-axis (float)
        enc -> encoder count (int)
        wall -> whether a wall is detected in front (bool)
        reached -> whether the distance given has been covered by the bot"""

    yaw = 0
    enc = 0
    wall = False
    reached = False

    client_socket.sendto("Sensors".encode(), address)  # Request data from arduino

    rec_data, addr = client_socket.recvfrom(2048)  # Read response from arduino
    print("Received " + rec_data.decode() + " from " + addr[0])  # Print the response from Arduino

    if addr == address:
        r = parse("{:f} {:d} {:d} {:d}", rec_data.decode())
        yaw = r[0]
        enc = r[1]
        wall = r[2]
        reached = r[3]

    return yaw, enc, wall, reached


def sendData(distance, direction):
    """ Send the next distance and direction to arduino"""

    data = "{} {}".format(direction, distance)
    print("Sending", data)
    client_socket.sendto(data.encode(), address)  # send command to arduino

    return

# sudo chmod 766 /dev/ttyUSB0
'''
Plotter module making a 2D graph of the data retrieved
from the Lidar sensor

'''
import math
import time

import matplotlib.pyplot as plt

from LidarX2 import LidarX2


def draw(x_data, y_data):
    """
        Plot the x and y lists of points on the graph
        :param x_data:
        :param y_data:
        :return:
    """
    plt.figure(1)
    plt.cla()
    plt.ylim(-9000, 9000)
    plt.xlim(-9000, 9000)
    plt.scatter(x_data, y_data, c='r', s=8)
    plt.pause(0.001)


lidar = LidarX2("/dev/ttyUSB0")  # PyLidar3.your_version_of_lidar(port,chunk_size)

if lidar.open():
    t = time.time()  # start time
    x, y = [0] * 360, [0] * 360

    while (time.time() - t) < 60:  # scan for 30 seconds
        data = lidar.get_measures()
        for binome in data:
            angle, distance = int(binome.angle), binome.distance
            print(f"angle: {angle}, distance: {distance}")
            x[angle] = distance * math.cos(math.radians(angle))
            y[angle] = distance * math.sin(math.radians(angle))
        draw(x, y)

    plt.close("all")
    lidar.close()
else:
    print("Error connecting to device")

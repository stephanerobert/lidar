"""
Sample code to read the Lidar Sensor
"""
import time

from LidarX2 import LidarX2

lidar = LidarX2("/dev/ttyUSB0")  # "/dev/cu.SLAB_USBtoUART")  # Name of the serial port, can be /dev/tty*, COM*, etc.

if not lidar.open():
    print("Cannot open lidar")
    exit(1)

time.sleep(1)
measures = lidar.get_measures()  # Get latest lidar measures
print(measures)

lidar.close()

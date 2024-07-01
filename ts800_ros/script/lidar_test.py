import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt

def scan_callback(msg):
    global x_coords, y_coords, ds

    #
    angle_min = msg.angle_min
    angle_increment = msg.angle_increment
    #
    range_min = msg.range_min
    ranges = msg.ranges

    for i in range(len(ranges)):
        if ranges[i] > range_min:
            angle = angle_min + i * angle_increment
            if not np.isinf(angle) and not np.isnan(angle):
                x = ranges[i] * np.cos(angle)
                y = ranges[i] * np.sin(angle)
                d = ranges[i]
                #rint("i: %d X: %f and Y: %f" % (i, x, y))
                #if not np.isnan(x) and not np.isnan(y):
                x_coords.append(x)
                y_coords.append(y)
                ds.append(d)

rospy.init_node('lidar_plotter')

x_coords = []
y_coords = []
ds = []

rospy.Subscriber('/scan', LaserScan, scan_callback)
rospy.spin()

plt.scatter(x_coords, y_coords, s=1)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Lidar Scan Data')
plt.gca().set_aspect('equal', adjustable='box')
plt.grid(True)

if len(x_coords) >= 4 and len(y_coords) >= 4:
    max_x = max(x_coords)
    min_x = min(x_coords)
    max_y = max(y_coords)
    min_y = min(y_coords)

    distance_top = max_y - min_y
    distance_bottom = max_y - min_y
    distance_left = max_x - min_x
    distance_right = max_x - min_x

    print("Top Distance:", distance_top)
    print("Bottom Distance:", distance_bottom)
    print("Left Distance:", distance_left)
    print("Right Distance:", distance_right)

plt.axhline(y=min_y, color='r', linestyle='--')  # Bottom line
plt.axhline(y=max_y, color='r', linestyle='--')  # Top line
plt.axvline(x=min_x, color='g', linestyle='--')  # Left line
plt.axvline(x=max_x, color='g', linestyle='--')  # Right line

plt.show()
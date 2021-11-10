#!/usr/bin/env python

import rospy, copy, math
from threading import Lock
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32
import rosbag
import sys
import numpy as np
import rosbag
import roslib 
import matplotlib
from matplotlib import pyplot as plt
from matplotlib.patches import Circle
from matplotlib.patches import Rectangle
from math import radians, cos, sin, asin, sqrt
import time
from scipy import signal
from scipy import fft
xrange = range

from sensor_msgs.msg import Imu
from mpl_toolkits import mplot3d
matplotlib.rcParams['interactive'] == True


bag_file = sys.argv[1]

#types = {}
#for bagname in sys.argv[1:]:
#    bag = rosbag.Bag(bagname)
#    for topic, msg, t in bag.read_messages():
#        if not msg._type in types:
#            types[msg._type] = msg._full_text
#
#for t in types:
#    print("Message type:", t)
#    print("Message text:")
#    print(types[t])

topic = ["/imu/data", "/odometry/filtered", "/odometry/gps", "/os_cloud_node/imu", "/piksi_imu/imu", "/piksi_imu/mag"]
'''
if len(sys.argv) < 3:
	topic[0] = "/imu/data"
else:
	topic = sys.argv[2]
#imu_data_linear = np.array[]
'''
bag = rosbag.Bag(bag_file)
msg_num = bag.get_message_count()

imu_data_linear = []
imu_data_linear_magnitude = []
imu_data_angular_magnitude = []
imu_data_angular = []

rtk_data = []
magnetic_field = []
odometry_heading = []
odometry_position = []

os_imu_linear = []
os_imu_angular = []
os_imu_linear_magnitude = []
os_imu_angular_magnitude = []

piksi_raw_linear = []
piksi_raw_angular = []
piksi_raw_linear_magnitude = []
piksi_raw_angular_magnitude = []

speed_actual = []
brake = []
throttle = []

waypoints = [] # msg::Path from avt_341/waypoints

for topic, msg, t in bag.read_messages(topics=['/imu/data']):
	imu_data_linear.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z, msg.header.stamp.to_sec()])
	imu_data_angular.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, msg.header.stamp.to_sec()])
	imu_data_linear_magnitude.append([abs(msg.linear_acceleration.x)+abs(msg.linear_acceleration.y)+abs(msg.linear_acceleration.z), msg.header.stamp.to_sec()])
	imu_data_angular_magnitude.append([abs(msg.angular_velocity.x)+abs(msg.angular_velocity.y)+abs(msg.angular_velocity.z), msg.header.stamp.to_sec()])

for topic, msg, t in bag.read_messages(topics=['/os_cloud_node/imu']):
	os_imu_linear.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z, msg.header.stamp.to_sec()])
	os_imu_angular.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, msg.header.stamp.to_sec()])
	os_imu_angular_magnitude.append([abs(msg.angular_velocity.x)+abs(msg.angular_velocity.y)+abs(msg.angular_velocity.z), msg.header.stamp.to_sec()])
	os_imu_linear_magnitude.append([abs(msg.linear_acceleration.x)+abs(msg.linear_acceleration.y)+abs(msg.linear_acceleration.z), msg.header.stamp.to_sec()])

for topic, msg, t in bag.read_messages(topics=['/piksi_imu/imu']):
	piksi_raw_linear.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z, msg.header.stamp.to_sec()])
	piksi_raw_angular.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, msg.header.stamp.to_sec()])
	piksi_raw_linear_magnitude.append([abs(msg.linear_acceleration.x)+abs(msg.linear_acceleration.y)+abs(msg.linear_acceleration.z), msg.header.stamp.to_sec()])
	piksi_raw_angular_magnitude.append([abs(msg.angular_velocity.x)+abs(msg.angular_velocity.y)+abs(msg.angular_velocity.z), msg.header.stamp.to_sec()])


def moving_average(array, window_size):
	moving_average = []
	i = 0
	while i < len(array) - window_size + 1:
		window = array[i : i+window_size]
		average = sum(window) / window_size
		moving_average.append(average)
		i+= 1
	return(moving_average)

imu_data_linear = np.array(imu_data_linear).transpose()
imu_data_angular = np.array(imu_data_angular).transpose()
os_imu_linear = np.array(os_imu_linear).transpose()
os_imu_angular = np.array(os_imu_angular).transpose()
piksi_raw_linear = np.array(piksi_raw_linear).transpose()
piksi_raw_angular = np.array(piksi_raw_angular).transpose()

imu_data_linear_magnitude = np.array(imu_data_linear_magnitude).transpose()
imu_data_angular_magnitude = np.array(imu_data_angular_magnitude).transpose()
os_imu_linear_magnitude = np.array(os_imu_linear_magnitude).transpose()
os_imu_angular_magnitude = np.array(os_imu_angular_magnitude).transpose()
piksi_raw_linear_magnitude = np.array(piksi_raw_linear_magnitude).transpose()
piksi_raw_angular_magnitude = np.array(piksi_raw_angular_magnitude).transpose()



# This is the wavelet, 
widths = np.arange(1, 100)
cwtmatr = signal.cwt(piksi_raw_linear[0], signal.ricker, widths)
plt.imshow(cwtmatr, extent=[-1, 1, 1, 100], cmap='PRGn', aspect='auto', vmax=abs(cwtmatr).max(), vmin=-abs(cwtmatr).max())

'''
average_window = 10
for signal in imu_data_linear[0]:
	signal_average = signal[0]
'''
#find the frequency of the data publisher for the FFT

fig, axs = plt.subplots(2, 3)
### TODO: These should be plotted based on time rather than frames. It would avoid the
###       funky scaling that I'm having to do. 
arr = list(xrange(0, len(piksi_raw_linear[0])))
arr2 = list(xrange(0, len(imu_data_linear[0])))
arr = [element * (float(len(os_imu_linear[0])/float(len(piksi_raw_linear[0])))) for element in arr]
arr2 = [element * (float(len(os_imu_linear[0])/float(len(imu_data_linear[0])))) for element in arr2]
#axs[0, 0].plot(xrange(0, len(os_imu_linear[0])), os_imu_linear[0], label='OS')
#axs[0, 0].plot(arr, piksi_raw_linear[0], label='Piksi')
#axs[0, 0].plot(arr2, imu_data_linear[0], label='IMU')
axs[0, 0].plot(os_imu_linear[3], os_imu_linear[0], label='OS')
axs[0, 0].plot(piksi_raw_linear[3], piksi_raw_linear[0], label='Piksi')
axs[0, 0].plot(imu_data_linear[3], imu_data_linear[0], label='IMU')

axs[0, 0].legend(loc ='upper right')
axs[0, 0].set_title('Raw Linear X (Forward)')
axs[0, 0].set_ylabel("Meters Per Second")
#axs[0, 1].plot(xrange(0, len(os_imu_linear[1])), os_imu_linear[1], label='OS')
#axs[0, 1].plot(arr, piksi_raw_linear[1], label='Piksi')
#axs[0, 1].plot(arr2, imu_data_linear[1], label='IMU')
axs[0, 1].plot(os_imu_linear[3], os_imu_linear[1], label='OS')
axs[0, 1].plot(piksi_raw_linear[3], piksi_raw_linear[1], label='Piksi')
axs[0, 1].plot(imu_data_linear[3], imu_data_linear[1], label='IMU')
axs[0, 1].legend(loc = 'upper right')
axs[0, 1].set_title('Raw Linear Y (Left)')
axs[0, 1].set_ylabel("Meters Per Second")
#axs[0, 2].plot(xrange(0, len(os_imu_linear[2])), os_imu_linear[2], label='OS')
#axs[0, 2].plot(arr, piksi_raw_linear[2], label='Piksi')
#axs[0, 2].plot(arr2, imu_data_linear[2], label='IMU')
axs[0, 2].plot(os_imu_linear[3], os_imu_linear[2], label='OS')
axs[0, 2].plot(piksi_raw_linear[3], piksi_raw_linear[2], label='Piksi')
axs[0, 2].plot(imu_data_linear[3], imu_data_linear[2], label='IMU')
axs[0, 2].legend(loc = 'upper right')
axs[0, 2].set_title('Raw Linear Z (Up)')
axs[1, 0].plot(os_imu_angular[3], os_imu_angular[0], label='OS')
axs[1, 0].plot(piksi_raw_angular[3], piksi_raw_angular[0], label='Piksi')
axs[1, 0].plot(imu_data_angular[3], imu_data_angular[0], label='IMU')
axs[1, 0].set_ylabel("Degrees Per Second")
axs[1, 0].legend(loc ='upper right')
axs[1, 0].set_title('Raw Angular X (Roll)')
axs[1, 1].plot(os_imu_angular[3], os_imu_angular[1], label='OS')
axs[1, 1].plot(piksi_raw_angular[3], piksi_raw_angular[1], label='Piksi')
axs[1, 1].plot(imu_data_angular[3], imu_data_angular[1], label='IMU')
axs[1, 1].set_ylabel("Degrees Per Second")
axs[1, 1].legend(loc ='upper right')
axs[1, 1].set_title('Raw Angular Y (Pitch)')
axs[1, 2].plot(os_imu_angular[3], os_imu_angular[2], label='OS')
axs[1, 2].plot(piksi_raw_angular[3], piksi_raw_angular[2], label='Piksi')
axs[1, 2].plot(imu_data_angular[3], imu_data_angular[2], label='IMU')
axs[1, 2].set_ylabel("Degrees Per Second")
axs[1, 2].legend(loc ='upper right')
axs[1, 2].set_title('Raw Angular Z (Heading)')

fig, axs = plt.subplots(2, 3)
axs[0, 0].plot(os_imu_linear_magnitude[1], os_imu_linear_magnitude[0], label='OS Imu Linear magnitude')
axs[0, 1].plot(piksi_raw_linear_magnitude[1], piksi_raw_linear_magnitude[0], label='Piksi Linear Magnitude')
axs[0, 2].plot(imu_data_linear_magnitude[1], imu_data_linear_magnitude[0], label='IMU Linear Magnitude')

axs[0, 0].set_title('OS Imu Linear magnitude')
axs[0, 0].set_ylabel("Meters Per Second")

axs[0, 1].set_title('Piksi Raw Imu Linear magnitude')
axs[0, 1].set_ylabel("Meters Per Second")

axs[0, 2].set_title('IMU Data Linear magnitude')
axs[0, 2].set_ylabel("Meters Per Second")

axs[1, 0].plot(os_imu_angular_magnitude[1], os_imu_angular_magnitude[0], label='OS Imu Angular magnitude')
axs[1, 1].plot(piksi_raw_angular_magnitude[1], piksi_raw_angular_magnitude[0], label='Piksi Angular Magnitude')
axs[1, 2].plot(imu_data_angular_magnitude[1], imu_data_angular_magnitude[0], label='IMU Angular Magnitude')

axs[1, 0].set_title('OS Imu Angular magnitude')
axs[1, 0].set_ylabel("Meters Per Second")

axs[1, 1].set_title('Piksi Raw Imu Angular magnitude')
axs[1, 1].set_ylabel("Meters Per Second")

axs[1, 2].set_title('IMU Data Angular magnitude')
axs[1, 2].set_ylabel("Meters Per Second")


fig, axs = plt.subplots(2, 3)
axs[0, 0].plot(fft(os_imu_linear_magnitude[0]), label='OS FFT')
axs[0, 0].plot(fft(piksi_raw_linear_magnitude[0]), label='Piksi FFT')
axs[0, 0].plot(fft(imu_data_linear_magnitude[0]), label='IMU FFT')

axs[0, 0].legend(loc ='upper right')
axs[0, 0].set_title('FFT on Magnitude Linear')
#axs[0, 1].plot(xrange(0, len(os_imu_linear[1])), os_imu_linear[1], label='OS')
#axs[0, 1].plot(arr, piksi_raw_linear[1], label='Piksi')
#axs[0, 1].plot(arr2, imu_data_linear[1], label='IMU')
axs[0, 1].plot(fft(os_imu_angular_magnitude[0]), label='OS')
axs[0, 1].plot(fft(piksi_raw_angular_magnitude[0]), label='Piksi')
axs[0, 1].plot(fft(imu_data_angular_magnitude[0]), label='IMU')
axs[0, 1].legend(loc = 'upper right')
axs[0, 1].set_title('FFT On Magnitude Angular')

#axs[0, 2].plot(xrange(0, len(os_imu_linear[2])), os_imu_linear[2], label='OS')
#axs[0, 2].plot(arr, piksi_raw_linear[2], label='Piksi')
#axs[0, 2].plot(arr2, imu_data_linear[2], label='IMU')
axs[0, 2].plot(os_imu_linear[3], fft(os_imu_linear[2]), label='OS')
axs[0, 2].plot(piksi_raw_linear[3], fft(piksi_raw_linear[2]), label='Piksi')
axs[0, 2].plot(imu_data_linear[3], fft(imu_data_linear[2]), label='IMU')
axs[0, 2].legend(loc = 'upper right')
axs[0, 2].set_title('FFT On Linear Z (Up)')
axs[1, 0].plot(os_imu_angular[3], fft(os_imu_angular[0]), label='OS')
axs[1, 0].plot(piksi_raw_angular[3], fft(piksi_raw_angular[0]), label='Piksi')
axs[1, 0].plot(imu_data_angular[3], fft(imu_data_angular[0]), label='IMU')

axs[1, 0].legend(loc ='upper right')
axs[1, 0].set_title('FFT On Angular X (Roll)')
axs[1, 1].plot(os_imu_angular[3], fft(os_imu_angular[1]), label='OS')
axs[1, 1].plot(piksi_raw_angular[3], fft(piksi_raw_angular[1]), label='Piksi')
axs[1, 1].plot(imu_data_angular[3], fft(imu_data_angular[1]), label='IMU')

axs[1, 1].legend(loc ='upper right')
axs[1, 1].set_title('FFT On Angular Y (Pitch)')
axs[1, 2].plot(os_imu_angular[3], os_imu_angular[2], label='OS')
axs[1, 2].plot(piksi_raw_angular[3], piksi_raw_angular[2], label='Piksi')
axs[1, 2].plot(imu_data_angular[3], imu_data_angular[2], label='IMU')

axs[1, 2].legend(loc ='upper right')
axs[1, 2].set_title('FFT Angular Z (Heading)')

plt.show()


# utilities used for calibration of dvrk-psm-force feedback device
import dvrk
import rospy
import math
import matplotlib.pyplot as plt
from rosserial_arduino.msg import Adc
from sensor_msgs.msg import PointCloud
import numpy as np


def distance(p1, p2):
    return math.sqrt(((p2.x-p1.x)**2)+((p2.y-p1.y)**2)+((p2.z-p1.z)**2))


def find_unit_vector(p1, p2):
    return (p2 - p1)/np.linalg.norm(p2 - p1)


def make_fig(label, x_list, y_list, i, length, positioner):
    plt.subplot(positioner)
    plt.xlabel('sample')
    plt.ylabel(label)
    plt.grid(True)
    plt.title(label)
    plt.xlim((i - length, i))
    plt.plot(x_list, y_list)


# create subscriber for data from Polaris(Optical Tracking Camera)
class OpticalTracker:

    def __init__(self):
        self.msg = None
        rospy.Subscriber('/ndi/fiducials', PointCloud, self.callback, queue_size=10)

    def callback(self, data):
        # filter data before publishing
        # if points number is not equal to 2 ignore the data
        if len(data.points) is 2:
            # if point moved too far ignore the data
            # if distance(data.points[0], data.points[1]) < 5*0.0508:
            self.msg = data
            self.msg.header.frame_id = "world"
            # else:
            #     self.msg = None
            #     print("Moved too far")
        else:
            self.msg = None
            print("Invalid number of points")

    def get_ot_data(self):
        return self.msg

    def get_point_data(self, point_num):
        return self.msg.points[point_num]


# create class for data listener of z-direction and load cell data from PCB with red LED
class ZLCdataFromADC:

    def __init__(self, adc_number):
        self.data_value = 0
        self.ADC_number = adc_number
        self.ADC_DATA_ARR = []
        rospy.Subscriber('/adc_zlc', Adc, self.callback)

    def callback(self, data):
        self.ADC_DATA_ARR = [data.adc0, data.adc1, data.adc2, data.adc3, data.adc4]
        self.data_value = self.ADC_DATA_ARR[self.ADC_number]

    def get_value(self):
        return self.data_value

    def display_number(self):
        print self.ADC_number


# create class for data listener of x-direction and y-direction data from PCB with orange LED
class XYdataFromADC:

    def __init__(self, adc_number):
        self.data_value = 0
        self.ADC_number = adc_number
        self.ADC_DATA_ARR = []
        rospy.Subscriber('/adc_xy', Adc, self.callback)

    def callback(self, data):
        self.ADC_DATA_ARR = [data.adc0, data.adc1, data.adc2, data.adc3, data.adc4]
        self.data_value = self.ADC_DATA_ARR[self.ADC_number]

    def get_value(self):
        return self.data_value

    def display_number(self):
        print self.ADC_number



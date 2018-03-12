import rospy
import math
import matplotlib.pyplot as plt
from rosserial_arduino.msg import Adc
from sensor_msgs.msg import PointCloud


def distance(p1, p2):
    return math.sqrt(((p2.x-p1.x)**2)+((p2.y-p1.y)**2)+((p2.z-p1.z)**2))


def force_transform_x(p1, p2, Fm):
    return Fm*unit_vector_x(p1, p2)


def unit_vector_x(p1, p2):
    return (p2.x - p1.x)/distance(p1, p2)


def force_transform_y(p1, p2, Fm):
    return Fm*unit_vector_y(p1, p2)


def unit_vector_y(p1, p2):
    return (p2.y - p1.y)/distance(p1, p2)


def force_transform_z(p1, p2, Fm):
    return Fm*unit_vector_z(p1, p2)


def unit_vector_z(p1, p2):
    return (p2.z - p1.z)/distance(p1, p2)


def make_fig(label, x_list, y_list, i, length):
    plt.xlabel('sample')
    plt.ylabel(label)
    plt.grid(True)
    plt.title(label)
    plt.xlim((i - length, i))
    plt.plot(x_list, y_list)


# create subscriber for data from Polaris Optical Tracking Data
class OpticalTracker:

    def __init__(self):
        self.msg = None
        rospy.Subscriber('/ndi/fiducials', PointCloud, self.callback, queue_size=10)

    def callback(self, data):
        # filter data before publishing
        # if points number is not equal to 2 ignore the data
        if len(data.points) is 2:
            # if point moved too far ignore the data
            if distance(data.points[0], data.points[1]) < 5*0.0508:
                self.msg = data
                self.msg.header.frame_id = "world"
            else:
                print("Moved too far")
        else:
            print("Invalid number of points")

    def get_ot_data(self):
        return self.msg

    def get_point_data(self, point_num):
        return self.msg.points[point_num]


class ZLCdataFromADC:

    def __init__(self, adc_number):
        self.data_value = 0
        self.ADC_number = adc_number
        self.ADC_DATA_ARR = []
        rospy.Subscriber('/adc', Adc, self.callback)

    def callback(self, data):
        self.ADC_DATA_ARR = [data.adc0, data.adc1, data.adc2, data.adc3, data.adc4]
        self.data_value = self.ADC_DATA_ARR[self.ADC_number]

    def get_value(self):
        return self.data_value

    def display_number(self):
        print self.ADC_number


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



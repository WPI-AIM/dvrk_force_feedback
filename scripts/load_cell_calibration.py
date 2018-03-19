## Simple talker demo that listens to std_msgs/Strings published
## to the 'chatter' topic

from datetime import datetime
import rospy
import numpy as np
import matplotlib.pyplot as plt
from rosserial_arduino.msg import Adc

lc_ave_arr = []
lc_ave_plot = []
force_arr = []
answer = '0'
fname = 'data'
weight = 0
force = 0
y_max_limit = 0
y_min_limit = 0


class WheatstoneBridge:

    def __init__(self, ADC_number):
        self.data_arr = []
        self.index = 0
        self.ADC_number = ADC_number
        self.ADC_DATA_ARR = []
        rospy.Subscriber('/adc_zlc', Adc, self.callback)

    def callback(self, data):
        self.ADC_DATA_ARR = [data.adc0, data.adc1, data.adc2, data.adc3, data.adc4]
        if self.index < 10:
            self.data_arr.append(self.ADC_DATA_ARR[self.ADC_number])
            self.index = self.index + 1
        else:
            del self.data_arr[0]
            self.data_arr.append(self.ADC_DATA_ARR[self.ADC_number])

    def get_average(self):
        return np.mean(self.data_arr)

    def display_number(self):
        print self.ADC_number


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('adc_listener', anonymous=True)

    lc_force = WheatstoneBridge(0)

    condition = True
    # Ask user to type a filname
    fname = raw_input("Type a new file name: ")
    while condition:
        # Ask user "Add data point? (y/n)"
        answer = raw_input("Add data point? (y/n) ")
        # check if answer is yes
        if answer == 'y':
            weight = input("Type weight [g]: ")
            # Calculate force from the weight value
            force = (weight * 9.8) / 1000
            force_arr.append(force)
            # Measure average value of 100 readings from the first Wheatstone Bridge WB1
            lc_ave_arr.append(lc_force.get_average())

        elif answer == 'n':

            # Find parameters for linear equation for data fitting
            [a, b] = np.polyfit(lc_ave_arr, force_arr, 1)
            print "Equation parameter a=", a, ", b=", b
            np.savez('load_cell_linear_equation_parameters.npz', lc_equation_parameters=[a, b])

            # Make a plot Voltage Readings vs Force
            # Find maximum value of WB readings
            lc_ave_plot = [x - min(lc_ave_arr) for x in lc_ave_arr]
            y_max = max(lc_ave_plot)
            y_min = min(lc_ave_plot)
            plt.figure(1)

            plt.subplot(221)
            plt.plot(force_arr, lc_ave_plot, 'bo')
            plt.xlabel('Force, [N]')
            plt.ylabel('Output LC, [ADC]')
            plt.grid(True)
            plt.title('Force vs LC Readings')
            plt.ylim((y_min, y_max))

            fname_str = fname + str(datetime.now())
            print fname_str
            plot_name = fname_str + '.png'
            plt.savefig(plot_name)

            plt.show()
            condition = False
            # Save data in txt file
            thefile = open('/home/parallels/catkin_ws/src/force_feedback_anna/scripts/results/{0}.txt'.format(fname_str), 'a')
            thefile.write('Force LC \n')
            # loop through each item in the list
            # and write it to the output file
            for (f, r1) in zip(force_arr, lc_ave_arr):
                thefile.write('{} {} \n'.format(str(f), str(r1)))
            thefile.write('\n \n')
            thefile.close()

        else:
            print ("Try again!")
        # simply keeps python from exiting until this node is stopped
    # rospy.spin()


if __name__ == '__main__':
    try:
        listener()

    except rospy.ROSInterruptException:

        pass

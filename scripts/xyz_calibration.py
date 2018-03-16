#  Calibration of dvrk-psm-force-feedback device using optical tracking system and load cell
#  Collects data from load cell, device, and two optical trackers and gives output txt file with force
#  measured with device and force measured using load cell
#  real-time plotting of force measured from load cell

import utilities
import rospy
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
from datetime import datetime
from numpy.linalg import inv

size = 100
force_x_lc = 0
force_y_lc = 0
force_z_lc = 0
force_x_m = 0
force_y_m = 0
force_z_m = 0

# create arrays to save data
arr_force_x_lc = []
arr_force_y_ls = []
arr_force_z_lc = []
arr_force_x_m = []
arr_force_y_m = []
arr_force_z_m = []
arr_force_m = []
arr_point1_crd = []
arr_point2_crd = []

deq_x_data = deque(maxlen=size)

force_m = 0          # force magnitude measured from load cell
deq_force_m = deque(maxlen=size)
answer = '0'
condition = True


def publish_data():
    pub_fx_lc.publish(Float32(force_x_lc))
    pub_fy_lc.publish(Float32(force_y_lc))
    pub_fz_lc.publish(Float32(force_z_lc))
    pub_force_m.publish(Float32(force_m))
    pub_fx.publish(Float32(force_x_m))
    pub_fy.publish(Float32(force_y_m))
    pub_fz.publish(Float32(force_z_m))


def add_data_to_arr():
    arr_force_x_lc.append(force_x_lc)
    arr_force_y_ls.append(force_y_lc)
    arr_force_z_lc.append(force_z_lc)
    arr_force_x_m.append(force_x_m)
    arr_force_y_m.append(force_y_m)
    arr_force_z_m.append(force_z_m)
    arr_force_m.append(force_m)
    arr_point1_crd.append(p1)
    arr_point2_crd.append(p2)


if __name__ == '__main__':
    # create plot
    plt.ion()  # enable interactivity
    fig = plt.figure()  # make a figure

    # create node
    rospy.init_node('adc_listener', anonymous=True)

    # create classes
    lc = utilities.ZLCdataFromADC(0)
    z_adc = utilities.ZLCdataFromADC(1)
    x_adc = utilities.XYdataFromADC(0)
    y_adc = utilities.XYdataFromADC(1)
    opt_track = utilities.OpticalTracker()
    rate = rospy.Rate(50)

    # create publishers
    pub_fx_lc = rospy.Publisher('/adc_listener/force_x_lc', Float32, queue_size=1)
    pub_fy_lc = rospy.Publisher('/adc_listener/force_y_lc', Float32, queue_size=1)
    pub_fz_lc = rospy.Publisher('/adc_listener/force_z_lc', Float32, queue_size=1)
    pub_force_m = rospy.Publisher('/adc_listener/force_m', Float32, queue_size=1)
    pub_fx = rospy.Publisher('/adc_listener/force_x', Float32, queue_size=1)
    pub_fy = rospy.Publisher('/adc_listener/force_y', Float32, queue_size=1)
    pub_fz = rospy.Publisher('/adc_listener/force_z', Float32, queue_size=1)

    i = 0
    answer = 0
    npzfile = np.load("transformation_matrix.npz")
    trans_matrix = npzfile['transform']

    # while not rospy.is_shutdown():
    if opt_track.get_ot_data() is not None:
        while condition:
            answer = raw_input("Start calibration data collection? (y/n) ")
            if answer == 'y':
                for j in range(0, 300):
                    # update point number
                    i = i + 1
                    deq_x_data.append(i)

                    # use calibration equation to transform ADC values to Forces
                    force_m = (lc.get_value() - 31720)/3.017
                    deq_force_m.append(force_m)

                    # find position of two optical trackers
                    p1 = opt_track.get_point_data(0)
                    p2 = opt_track.get_point_data(1)

                    vec_p1 = np.array([p1.x, p1.y, p1.z])
                    vec_p2 = np.array([p2.x, p2.y, p2.z])

                    # change size to 1x4

                    # update positions using transformation matrix
                    trans_p1 = np.dot(trans_matrix, np.append(vec_p1, 1))
                    trans_p2 = np.dot(trans_matrix, np.append(vec_p2, 1))

                    # remove last element
                    trans_p1 = np.delete(trans_p1, 3)
                    trans_p2 = np.delete(trans_p2, 3)

                    # find forces in X,Y,Z direction using load cell data
                    unit_vector = utilities.unit_vector(p1, p2)

                    trans_force = np.dot(inv(trans_matrix), np.append(unit_vector, 1))
                    print "transformed forces", trans_force
                    force_x_lc = force_m*trans_force[0]
                    force_y_lc = force_m*trans_force[1]
                    force_z_lc = force_m*trans_force[2]

                    force_x_m = x_adc.get_value()
                    force_y_m = y_adc.get_value()
                    force_z_m = z_adc.get_value()

                    publish_data()
                    add_data_to_arr()

                    # real-time plotting of Force magnitude data
                    if i > 100:
                        utilities.make_fig('Force Magnitude [mN]', deq_x_data, deq_force_m, i, 100)
                        plt.pause(0.001)
                    # rate.sleep()
            elif answer == 'n':
                condition = False
                # Save data in txt file
                fname_str = 'calibration_data' + str(datetime.now())
                print fname_str
                plot_name = fname_str + '.png'
                plt.savefig(plot_name)
                thefile = open('{0}.txt'.format(fname_str), 'a')
                thefile.write('F_LC_x F_x F_LC_y F_y F_LC_z F_z F_m \n')
                # loop through each item in the list
                # and write it to the output file
                for (f_lc_x, f_x, f_lc_y, f_y, f_lc_z, f_z, f_m) \
                        in zip(arr_force_x_lc, arr_force_x_m, arr_force_y_ls,
                               arr_force_y_m, arr_force_z_lc, arr_force_z_m,
                               arr_force_m):
                    thefile.write('{} {} {} {} {} {} {} \n'.format(str(f_lc_x), str(f_x), str(f_lc_y),
                                                                   str(f_y), str(f_lc_z), str(f_z), str(f_m)))
                thefile.write('\n \n')
                thefile.close()
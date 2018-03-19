#  Calibration of dvrk-psm-force-feedback device using optical tracking system and load cell
#  Collects data from load cell, device, and two optical trackers and gives output txt file with force
#  measured with device and force measured using load cell
#  real-time plotting of force measured from load cell

import utilities
import rospy
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
arr_force_y_lc = []
arr_force_z_lc = []
arr_force_x_m = []
arr_force_y_m = []
arr_force_z_m = []
arr_force_m = []

# create deques for real-time plotting
deq_x_data = deque(maxlen=size)
deq_force_m = deque(maxlen=size)

force_m = 0          # force magnitude measured from load cell

answer = '0'
condition = True


def add_data_to_arr():
    arr_force_x_lc.append(force_x_lc)
    arr_force_y_lc.append(force_y_lc)
    arr_force_z_lc.append(force_z_lc)
    arr_force_x_m.append(force_x_m)
    arr_force_y_m.append(force_y_m)
    arr_force_z_m.append(force_z_m)
    arr_force_m.append(force_m)


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

    i = 0
    answer = 0
    # load transformation matrix from npz file
    npzfile = np.load("transformation_matrix.npz")
    trans_matrix = npzfile['transform']

    npz_lc_data = np.load("load_cell_linear_equation_parameters.npz")
    lc_lin_eq_param = npz_lc_data['lc_equation_parameters']

    while opt_track.get_ot_data() is None:
        while condition:
            answer = raw_input("Start calibration data collection (cover optical markers on the mount)? (y/n) ")
            if answer == 'y':
                for j in range(0, 300):
                    # update point number
                    i = i + 1
                    deq_x_data.append(i)

                    # use calibration equation to transform ADC values to Forces
                    force_m = lc_lin_eq_param[0]*lc.get_value() + lc_lin_eq_param[1]
                    deq_force_m.append(force_m)

                    # find position of two optical trackers
                    p1 = opt_track.get_point_data(0)
                    p2 = opt_track.get_point_data(1)

                    # find forces in X,Y,Z direction using load cell data
                    unit_vector = utilities.unit_vector(p1, p2)

                    # find unit vector in new coordinates
                    trans_unit_vec = np.dot(inv(trans_matrix), np.append(unit_vector, 1))

                    # find forces in each direction
                    force_x_lc = force_m*trans_unit_vec[0]
                    force_y_lc = force_m*trans_unit_vec[1]
                    force_z_lc = force_m*trans_unit_vec[2]

                    # measure force from force-feedback device
                    force_x_m = x_adc.get_value()
                    force_y_m = y_adc.get_value()
                    force_z_m = z_adc.get_value()

                    add_data_to_arr()

                    # real-time plotting of Force magnitude data
                    if i > 100:
                        utilities.make_fig('Force Magnitude [mN]', deq_x_data, deq_force_m, i, 100)
                        plt.pause(0.001)

            elif answer == 'n':
                # Find parameters for linear equation for data fitting
                [x_a, x_b] = np.polyfit(arr_force_x_m, arr_force_x_lc, 1)
                [y_a, y_b] = np.polyfit(arr_force_y_m, arr_force_y_lc, 1)
                [z_a, z_b] = np.polyfit(arr_force_z_m, arr_force_z_lc, 1)
                print "Equation parameter in x dir a=", x_a, ", b=", x_b
                print "Equation parameter in y dir a=", y_a, ", b=", y_b
                print "Equation parameter in z dir a=", z_a, ", b=", z_b
                parameters = [[x_a, x_b], [y_a, y_b], [z_a, z_b]]
                np.savez('xyz_linear_equation_parameters.npz', xyz_equation_parameters=parameters)

                condition = False
                # Save data in txt file
                fname_str = 'calibration_data' + str(datetime.now())
                print fname_str
                plot_name = fname_str + '.png'
                plt.savefig(plot_name)
                thefile = open('/home/parallels/catkin_ws/src/force_feedback_anna/scripts/results/{0}.txt'.format(fname_str), 'a')
                thefile.write('F_LC_x F_x F_LC_y F_y F_LC_z F_z F_m \n')
                # loop through each item in the list
                # and write it to the output file
                for (f_lc_x, f_x, f_lc_y, f_y, f_lc_z, f_z, f_m) \
                        in zip(arr_force_x_lc, arr_force_x_m, arr_force_y_lc,
                               arr_force_y_m, arr_force_z_lc, arr_force_z_m,
                               arr_force_m):
                    thefile.write('{} {} {} {} {} {} {} \n'.format(str(f_lc_x), str(f_x), str(f_lc_y),
                                                                   str(f_y), str(f_lc_z), str(f_z), str(f_m)))
                thefile.write('\n \n')
                thefile.close()
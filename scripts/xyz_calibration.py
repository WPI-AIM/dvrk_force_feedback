#  Calibration of dvrk-psm-force-feedback device using optical tracking system and load cell
#  Collects data from load cell, device, and two optical trackers and gives output txt file with force
#  measured with device and force measured using load cell
#  real-time plotting of force measured from load cell

import dvrk
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
deq_force_x = deque(maxlen=size)
deq_force_y = deque(maxlen=size)
deq_force_z = deque(maxlen=size)

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

    # Create a Python proxy for PSM1, name must match ros namespace
    p = dvrk.psm('PSM2')
    #p.home()
    #p.move_joint_some(np.array([0.23520355, -0.70646788,  0.1164548,  1.09101678]), np.array([0, 1, 2, 3]))
    # 0.1-0.24 3rd number coordinate of joint states (insertion joint)
    # create classes
    lc = utilities.ZLCdataFromADC(0)
    z_adc = utilities.ZLCdataFromADC(1)
    x_adc = utilities.XYdataFromADC(1)
    y_adc = utilities.XYdataFromADC(0)
    opt_track = utilities.OpticalTracker()
    rate = rospy.Rate(100)

    i = 0
    answer = 0
    # load transformation matrix from npz file
    npzfile = np.load("transformation_matrix.npz")
    transform_camera_to_robot = npzfile['transform']
    print 'trans matrix', transform_camera_to_robot

    npz_lc_data = np.load("load_cell_linear_equation_parameters.npz")
    lc_lin_eq_param = npz_lc_data['lc_equation_parameters']
    while condition:
        while opt_track.get_ot_data() is not None and condition is True:

            p1 = opt_track.get_point_data(0)
            p2 = opt_track.get_point_data(1)
            p1_array = np.array([p1.x, p1.y, p1.z])
            p2_array = np.array([p2.x, p2.y, p2.z])
            p1_robot = np.dot(inv(transform_camera_to_robot), np.append(p1_array, 1))
            p2_robot = np.dot(inv(transform_camera_to_robot), np.append(p2_array, 1))

            if p1_robot[2] > p2_robot[2]:
                p_stat_def = p1_robot
            else:
                p_stat_def = p2_robot

            answer = raw_input("Start calibration data collection (cover optical markers on the mount)? (y/n) ")
            if answer == 'y':
                for j in range(0, 500):

                    if opt_track.get_ot_data() is not None:

                        # find position of two optical trackers
                        p1 = opt_track.get_point_data(0)
                        p2 = opt_track.get_point_data(1)
                        p1_array = np.array([p1.x, p1.y, p1.z])
                        p2_array = np.array([p2.x, p2.y, p2.z])
                        p1_robot = np.dot(inv(transform_camera_to_robot), np.append(p1_array, 1))
                        p2_robot = np.dot(inv(transform_camera_to_robot), np.append(p2_array, 1))

                        if np.linalg.norm(p_stat_def[0:3]-p1_robot[0:3]) < np.linalg.norm(p_stat_def[0:3]-p2_robot[0:3]):
                            p_stat = p1_robot
                            p_mob = p2_robot
                        else:
                            p_stat = p2_robot
                            p_mob = p1_robot

                        # use calibration equation to transform ADC values to Forces
                        force_m = lc_lin_eq_param[0] * lc.get_value() + lc_lin_eq_param[1]
                        deq_force_m.append(force_m)

                        # update point number
                        i = i + 1
                        deq_x_data.append(i)

                        print "p1", p1_array, "p2", p2_array
                        print "p1_r", p_stat, "p2_r", p_mob

                        # find forces in X,Y,Z direction using load cell data
                        unit_vector = utilities.find_unit_vector(p_stat, p_mob)
                        print "unit vector", unit_vector

                        # find unit vector in new coordinates
                        #trans_unit_vec = np.dot(inv(transform_camera_to_robot), np.append(unit_vector, 1))

                        # find forces in each direction
                        force_x_lc = force_m*unit_vector[0]
                        force_y_lc = force_m*unit_vector[1]
                        force_z_lc = force_m*unit_vector[2]

                        deq_force_x.append(force_x_lc)
                        deq_force_y.append(force_y_lc)
                        deq_force_z.append(force_z_lc)

                        # measure force from force-feedback device
                        force_x_m = x_adc.get_value()
                        force_y_m = y_adc.get_value()
                        force_z_m = p.get_current_joint_effort()[2]
                        #force_z_m = z_adc.get_value()

                        add_data_to_arr()

                        # real-time plotting of Force magnitude data
                        if i > 100:
                            utilities.make_fig('Force Magnitude [N]', deq_x_data, deq_force_m, i, 100, 221)
                            utilities.make_fig('Force X [N]', deq_x_data, deq_force_x, i, 100, 222)
                            utilities.make_fig('Force Y [N]', deq_x_data, deq_force_y, i, 100, 223)
                            utilities.make_fig('Force Z [N]', deq_x_data, deq_force_z, i, 100, 224)
                            plt.pause(0.005)
                        rate.sleep()


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
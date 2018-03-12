#  Calibration of dvrk-psm-force-feedback device using optical tracking system and load cell
#  Collects data from load cell, device, and two optical trackers and gives output txt file with force
#  measured with device and force measured using load cell
#  real-time plotting of force measured from load cell

import utilities
import rospy
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from collections import deque
from datetime import datetime

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
    condition = True
    while not rospy.is_shutdown():
        if opt_track.get_ot_data() is not None:
            while condition:
                answer = raw_input("Start calibration data collection? (y/n) ")
                for j in range(0, 300):
                    if answer == 'y':
                        # update point number
                        i = i + 1
                        deq_x_data.append(i)

                        # use calibration equation to transform ADC values to Forces
                        force_m = (lc.get_value() - 31720)/3.017
                        deq_force_m.append(force_m)

                        # find position of two optical trackers
                        p1 = opt_track.get_point_data(0)
                        p2 = opt_track.get_point_data(1)

                        # find forces in X,Y,Z direction using load cell data
                        force_x_lc = utilities.force_transform_x(p1, p2, force_m)
                        force_y_lc = utilities.force_transform_y(p1, p2, force_m)
                        force_z_lc = utilities.force_transform_z(p1, p2, force_m)
                        force_x_m = x_adc.get_value()
                        force_y_m = y_adc.get_value()
                        force_z_m = z_adc.get_value()

                        # publish data
                        pub_fx_lc.publish(Float32(force_x_lc))
                        pub_fy_lc.publish(Float32(force_y_lc))
                        pub_fz_lc.publish(Float32(force_z_lc))
                        pub_force_m.publish(Float32(force_m))
                        pub_fx.publish(Float32(force_x_m))
                        pub_fy.publish(Float32(force_y_m))
                        pub_fz.publish(Float32(force_z_m))

                        # add data to arrays
                        arr_force_x_lc.append(force_x_lc)
                        arr_force_y_ls.append(force_y_lc)
                        arr_force_z_lc.append(force_z_lc)
                        arr_force_x_m.append(force_x_m)
                        arr_force_y_m.append(force_y_m)
                        arr_force_z_m.append(force_z_m)
                        arr_force_m.append(force_m)
                        arr_point1_crd.append(p1)
                        arr_point2_crd.append(p2)

                        # time.time() #     current time
                        # real-time plotting of Force magnitude data
                        if i > 100:
                            utilities.make_fig('Force Magnitude [mN]', deq_x_data, deq_force_m, i, 100)
                            plt.pause(0.001)
                    elif answer == 'n':
                        condition = False
                        # Save data in txt file
                        fname_str = 'calibration_data' + str(datetime.now())
                        print fname_str
                        plot_name = fname_str + '.png'
                        plt.savefig(plot_name)
                        thefile = open('{0}.txt'.format(fname_str), 'a')
                        thefile.write('F_LC_x F_x F_LC_y F_y F_LC_z F_z F_m p1 p2 \n')
                        # loop through each item in the list
                        # and write it to the output file
                        for (f_lc_x, f_x, f_lc_y, f_y, f_lc_z, f_z, f_m, p1, p2) \
                                in zip(arr_force_x_lc, arr_force_x_m, arr_force_y_ls,
                                       arr_force_y_m, arr_force_z_lc, arr_force_z_m,
                                       arr_force_m, arr_point1_crd, arr_point2_crd):
                            thefile.write('{} {} {} {} {} {} {} {} {} \n'.format(str(f_lc_x), str(f_x), str(f_lc_y),\
                                                                                 str(f_y), str(f_lc_z), str(f_z), \
                                                                                 str(f_m), str(p1), str(p2)))
                        thefile.write('\n \n')
                        thefile.close()

        rate.sleep()

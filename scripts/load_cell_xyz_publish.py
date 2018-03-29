#  Uses Polaris and two optical markers to find forces in x, y, z

import utilities
import rospy
import numpy as np
from numpy.linalg import inv
from std_msgs.msg import Float32


force_x = 0     # x-component of the force
force_y = 0     # y-component of the force
force_z = 0     # z-component of the force
force_m = 0     # force magnitude measured from the load cell


if __name__ == '__main__':
    # create node
    rospy.init_node('force_polaris_lc', anonymous=True)

    # create publishers
    pub_fx = rospy.Publisher('/force_polaris_lc/force_x', Float32, queue_size=1)
    pub_fy = rospy.Publisher('/force_polaris_lc/force_y', Float32, queue_size=1)
    pub_fz = rospy.Publisher('/force_polaris_lc/force_z', Float32, queue_size=1)

    # create classes
    lc = utilities.ZLCdataFromADC(0)
    opt_track = utilities.OpticalTracker()
    rate = rospy.Rate(500)

    # load transformation matrix from npz file
    npzfile = np.load("transformation_matrix.npz")
    transform_camera_to_robot = npzfile['transform']
    print 'trans matrix', transform_camera_to_robot

    npz_lc_data = np.load("load_cell_linear_equation_parameters.npz")
    lc_lin_eq_param = npz_lc_data['lc_equation_parameters']

    while opt_track.get_ot_data() is None:
        print 'No optical data yet'

    if opt_track.get_ot_data() is not None:
        # get position of optical markers in camera frame
        p1 = opt_track.get_point_data(0)
        p2 = opt_track.get_point_data(1)
        # convert positions into arrays
        p1_array = np.array([p1.x, p1.y, p1.z])
        p2_array = np.array([p2.x, p2.y, p2.z])
        # change coordinates of optical markers from camera frame to robot frame
        p1_robot = np.dot(inv(transform_camera_to_robot), np.append(p1_array, 1))
        p2_robot = np.dot(inv(transform_camera_to_robot), np.append(p2_array, 1))
        # find position of first marker (one that closer to origin point in z direction)
        if p1_robot[2] > p2_robot[2]:
            p_stat_def = p1_robot
        else:
            p_stat_def = p2_robot

    while not rospy.is_shutdown():

        if opt_track.get_ot_data() is not None:

            # find position of two optical trackers
            p1 = opt_track.get_point_data(0)
            p2 = opt_track.get_point_data(1)
            p1_array = np.array([p1.x, p1.y, p1.z])
            p2_array = np.array([p2.x, p2.y, p2.z])
            p1_robot = np.dot(inv(transform_camera_to_robot), np.append(p1_array, 1))
            p2_robot = np.dot(inv(transform_camera_to_robot), np.append(p2_array, 1))
            # find which optical marker is closer to static marker-> it is static marker
            if np.linalg.norm(p_stat_def[0:3]-p1_robot[0:3]) < np.linalg.norm(p_stat_def[0:3]-p2_robot[0:3]):
                p_stat = p1_robot
                p_mob = p2_robot
            else:
                p_stat = p2_robot
                p_mob = p1_robot

            # use calibration equation to transform ADC values to Forces
            force_m = lc_lin_eq_param[0] * lc.get_value() + lc_lin_eq_param[1]

            # find forces in each direction
            unit_vector = utilities.find_unit_vector(p_stat, p_mob)
            force_x = force_m*unit_vector[0]
            force_y = force_m*unit_vector[1]
            force_z = force_m*unit_vector[2]

            # publish data
            pub_fx.publish(Float32(force_x))
            pub_fy.publish(Float32(force_y))
            pub_fz.publish(Float32(force_z))

            rate.sleep()



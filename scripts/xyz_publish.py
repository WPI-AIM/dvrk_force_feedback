import utilities
from std_msgs.msg import Float32
import rospy
import numpy as np

force_x = 0
force_y = 0
force_z = 0


def convert_adc_to_force(adc_value, a, b):
    return a*adc_value + b


if __name__ == '__main__':
    rate = rospy.Rate(500)

    # create node
    rospy.init_node('force_feedback', anonymous=True)

    # instantiating classes
    x_adc = utilities.XYdataFromADC(1)
    y_adc = utilities.XYdataFromADC(0)
    z_adc = utilities.ZLCdataFromADC(1)

    # load linear equations parameters
    npz_xyz_data = np.load("xyz_linear_equation_parameters.npz")
    xyz_lin_eq_param = npz_xyz_data['xyz_equation_parameters']

    # create publishers
    pub_fx = rospy.Publisher('/force_feedback/force_x', Float32, queue_size=1)
    pub_fy = rospy.Publisher('/force_feedback/force_y', Float32, queue_size=1)
    pub_fz = rospy.Publisher('/force_feedback/force_z', Float32, queue_size=1)

    while not rospy.is_shutdown():

        # measure force from force-feedback device
        adc_x = x_adc.get_value()
        adc_y = y_adc.get_value()
        adc_z = z_adc.get_value()

        # convert data from ADC-values to force [mN]
        force_x = convert_adc_to_force(adc_x, xyz_lin_eq_param[0][0], xyz_lin_eq_param[0][1])
        force_y = convert_adc_to_force(adc_y, xyz_lin_eq_param[1][0], xyz_lin_eq_param[1][1])
        force_z = convert_adc_to_force(adc_z, xyz_lin_eq_param[2][0], xyz_lin_eq_param[2][1])

        # publish data
        pub_fx.publish(Float32(force_x))
        pub_fy.publish(Float32(force_y))
        pub_fz.publish(Float32(force_z))

        rate.sleep()
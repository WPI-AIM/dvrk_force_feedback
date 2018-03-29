import dvrk
import utilities
from std_msgs.msg import Float32
import rospy
import numpy as np

force_x = 0
force_y = 0
force_z = 0


def convert_adc_to_force(adc_value, a, b):
    return (adc_value - b)/a


if __name__ == '__main__':
    # Create a Python proxy for PSM2, name must match ros namespace
    p = dvrk.psm('PSM2')

    rate = rospy.Rate(500)

    # create node
    rospy.init_node('force_feedback', anonymous=True)

    # instantiating classes
    x_adc = utilities.XYdataFromADC(1)
    y_adc = utilities.XYdataFromADC(0)
    z_adc = utilities.ZLCdataFromADC(1)
    z_joint = p.get_current_joint_effort()[2]

    # # load linear equations parameters
    # npz_xyz_data = np.load("xyz_linear_equation_parameters.npz")
    # xyz_lin_eq_param = npz_xyz_data['xyz_equation_parameters']

    # create publishers
    pub_fx = rospy.Publisher('/force_feedback/force_x', Float32, queue_size=1)
    pub_fy = rospy.Publisher('/force_feedback/force_y', Float32, queue_size=1)
    pub_fz = rospy.Publisher('/force_feedback/force_z', Float32, queue_size=1)

    while not rospy.is_shutdown():

        position = p.get_current_joint_position()[2]
        # following numbers found from calibration
        a_x = -5924 * position + 1981
        b_x = 31360
        a_y = -12373 * position + 3520
        b_y = 33220
        a_z = 618
        b_z = 31172

        # measure force from force-feedback device
        adc_x = x_adc.get_value()
        adc_y = y_adc.get_value()
        adc_z = z_adc.get_value()

        # convert data from ADC-values to force [mN]
        force_x = convert_adc_to_force(adc_x, a_x, b_x)
        force_y = convert_adc_to_force(adc_y, a_y, b_y)
        force_z = convert_adc_to_force(adc_z, a_z, b_z)

        # publish data
        pub_fx.publish(Float32(force_x))
        pub_fy.publish(Float32(force_y))
        pub_fz.publish(Float32(force_z))

        rate.sleep()
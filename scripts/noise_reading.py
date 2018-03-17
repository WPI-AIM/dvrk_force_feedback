#  Reads noise from the PCBs for dvrk-psm-force-feedback

import utilities
import rospy
from datetime import datetime
from std_msgs.msg import Float32

size = 1000

# create arrays to save data
arr_data_1 = []
arr_data_2 = []


if __name__ == '__main__':
    # create node
    rospy.init_node('adc_listener', anonymous=True)
    pub_data_1 = rospy.Publisher('/adc_listener/data1', Float32, queue_size=1)
    pub_data_2 = rospy.Publisher('/adc_listener/data2', Float32, queue_size=1)

    # create classes
    ch_1 = utilities.XYdataFromADC(0)
    ch_2 = utilities.XYdataFromADC(1)

    answer = 0

    while not rospy.is_shutdown():
        for i in range(0, 1000):
            arr_data_1.append(ch_1.get_value())
            arr_data_2.append(ch_2.get_value())
            pub_data_1.publish(Float32(ch_1.get_value()))
            pub_data_2.publish(Float32(ch_2.get_value()))
        answer = raw_input("Write data? (y/n) ")
        if answer == 'y':
            fname_str = 'noise_data' + str(datetime.now())
            thefile = open('{0}.txt'.format(fname_str), 'a')
            thefile.write('1st_channel 2nd_channel\n')
            # loop through each item in the list
            # and write it to the output file
            for (c1, c2) in zip(arr_data_1, arr_data_2):thefile.write('{} {} \n'.format(str(c1), str(c2)))
            thefile.write('\n \n')
            thefile.close()

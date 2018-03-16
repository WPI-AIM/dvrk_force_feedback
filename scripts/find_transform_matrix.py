#  Calibration of dvrk-psm-force-feedback device using optical tracking system and load cell
#  Collects data from load cell, device, and two optical trackers and gives output txt file with force
#  measured with device and force measured using load cell
#  real-time plotting of force measured from load cell

import rospy
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import numpy as np
from sensor_msgs.msg import PointCloud
import math

answer = '0'
condition = True


# create subscriber for data from Polaris Optical Tracking Data
class OpticalTracker:

    def __init__(self):
        self.msg = None
        rospy.Subscriber('/ndi/fiducials', PointCloud, self.callback, queue_size=10)

    def callback(self, data):
        # filter data before publishing
        # if points number is not equal to 2 ignore the data
        if len(data.points) is 3:
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


def create_vector(p1, p2):
    return np.array(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z)


def distance(p1, p2):
    return math.sqrt(((p2.x-p1.x)**2)+((p2.y-p1.y)**2)+((p2.z-p1.z)**2))


if __name__ == '__main__':

    # create node
    rospy.init_node('opt_tracker', anonymous=True)

    # create classes
    opt_track = OpticalTracker()
    rate = rospy.Rate(50)

    answer = 0

    # while not rospy.is_shutdown():
    if opt_track.get_ot_data() is not None:
        while condition:
            answer = raw_input("Find transformation matrix (cover optical trackers on the spring)? (y/n) ")
            if answer == 'y':
                for j in range(0, 10):

                    # find position of optical markers
                    p1 = opt_track.get_point_data(0)
                    p2 = opt_track.get_point_data(1)
                    p3 = opt_track.get_point_data(2)

                    # find distances between optical markers
                    dis_1 = distance(p1, p2)
                    dis_2 = distance(p2, p3)
                    dis_3 = distance(p1, p3)

                    # create lists of three vectors
                    points = [(dis_1, p1, p2), (dis_2, p2, p3), (dis_3, p1, p3)]

                    # smallest distance is p1-p2, second small is p2-p3, largest is p3-p1
                    points.sort(key=lambda tup: tup[0])
                    shortest_vector = points[0]
                    second_vector = points[1]

                    # figure out which point is origin
                    if shortest_vector[1] == second_vector[1]:
                        p_origin = shortest_vector[1]
                        p_z = shortest_vector[2]
                        p_y = second_vector[2]
                    elif shortest_vector[2] == second_vector[2]:
                        p_origin = shortest_vector[2]
                        p_z = shortest_vector[1]
                        p_y = second_vector[1]
                    elif shortest_vector[1] == second_vector[2]:
                        p_origin = shortest_vector[1]
                        p_z = shortest_vector[2]
                        p_y = second_vector[1]
                    else:
                        p_origin = shortest_vector[2]
                        p_z = shortest_vector[1]
                        p_y = second_vector[2]

                    # find x,y,z
                    x_vec = create_vector(p1, p2)
                    y_vec = create_vector(p1, p2)



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
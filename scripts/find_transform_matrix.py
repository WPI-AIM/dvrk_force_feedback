#  Finds transformation matrix for 3 optical markers attached to the mount to calibrate
#  dvrk-psm-force-feedback device
# gives transformation_matrix.txt and transformation_matrix.npz files as an output

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud
import math

answer = '0'
condition = True

x_vec = []
y_vec = []
z_vec = []
origin_vec = []


# create subscriber for data from Polaris Optical Tracking Data
class OpticalTracker:

    def __init__(self):
        self.msg = None
        rospy.Subscriber('/ndi/fiducials', PointCloud, self.callback, queue_size=10)

    def callback(self, data):
        # filter data before publishing
        # if points number is not equal to 2 ignore the data
        if len(data.points) is 3:
            self.msg = data
            self.msg.header.frame_id = "world"
        else:
            print("Invalid number of points")

    def get_ot_data(self):
        return self.msg

    def get_point_data(self, point_num):
        return self.msg.points[point_num]


def create_vector(p2, p1):
    return normalize(np.array([p1.x-p2.x, p1.y-p2.y, p1.z-p2.z]))


def distance(p1, p2):
    return math.sqrt(((p2.x-p1.x)**2)+((p2.y-p1.y)**2)+((p2.z-p1.z)**2))


def normalize(input):
    return np.array(input / np.linalg.norm(input))


if __name__ == '__main__':

    # create node
    rospy.init_node('opt_tracker', anonymous=True)

    # create classes
    opt_track = OpticalTracker()
    rate = rospy.Rate(50)

    answer = 0
    homogeneous = None

    while opt_track.get_ot_data() is None:
            while condition:
                answer = raw_input("Find transformation matrix (cover optical trackers on the spring)? (y/n) ")
                if answer == 'y':

                    # find positions of optical markers
                    p1 = opt_track.get_point_data(0)
                    p2 = opt_track.get_point_data(1)
                    p3 = opt_track.get_point_data(2)

                    # find distances between optical markers
                    dis_1 = distance(p1, p2)
                    dis_2 = distance(p2, p3)
                    dis_3 = distance(p1, p3)

                    # create lists of three vectors
                    points = [(dis_1, p1, p2), (dis_2, p2, p3), (dis_3, p1, p3)]

                    # find smallest distance is p1-p2, second small is p2-p3, largest is p3-p1
                    points.sort(key=lambda tup: tup[0])
                    shortest_vector = points[0]
                    second_vector = points[1]

                    # figure out which point is which: origin (common between shortest and second vector), etc
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

                    # find new x,y,z vectors
                    z_vec = create_vector(p_origin, p_z)
                    y_vec = create_vector(p_origin, p_y)
                    x_vec = normalize(np.cross(y_vec, z_vec))
                    origin_vec = np.array([p_origin.x, p_origin.y, p_origin.z]).reshape((3, 1))

                    # create rotation matrix by merging x,y,z
                    rotation = np.concatenate((x_vec.reshape((3, 1)), y_vec.reshape((3, 1)), z_vec.reshape((3, 1))), axis=1)
                    # create homogeneous matrix by merging it with origin point coordinates
                    homogeneous = np.concatenate((np.concatenate((rotation, origin_vec), axis=1),
                                                  np.array([0, 0, 0, 1]).reshape((1, 4))), axis=0)

                elif answer == 'n':
                    condition = False
                    # Save data in npz file
                    np.savez('transformation_matrix.npz', transform=homogeneous)

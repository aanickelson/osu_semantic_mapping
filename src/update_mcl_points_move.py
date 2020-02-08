
import rospy

from nav_msgs.msg import Path, Odometry
import tf
import csv
import numpy as np
from math import pi

class OdomTransform:
    def __init__(self):
        self.last_msg = [0, 0, 0]
        self.this_msg = [0, 0, 0]
        self.diff = [0, 0, 0]

    def get_diff(self, msg):
        self.last_msg = self.this_msg
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.this_msg = [r, p, y]

        self.diff += [self.this_msg[0] - self.last_msg[0],
                     self.this_msg[1] - self.last_msg[1],
                     self.this_msg[2] - self.last_msg[2]]

    def translate_points(self, points):
        diff = np.array(self.diff + [0])
        for i in range(len(points)):
            points[i] = np.subtract(points[i], diff)

        # Reset the difference to zero
        self.diff = [0, 0, 0]


if __name__ == "__main__":
    odom = OdomTransform()
    rospy.init_node('update_mcl_points_py', anonymous=True)
    points = np.random.rand(100, 4)
    for i in range(len(points)):
        points[i][1] *= 5
        points[i][0] *= 5
        points[i][2] *= 2 * pi

    rospy.Subscriber("/odom", Odometry, odom.get_diff)

    odom.translate_points(points)

    rospy.spin()

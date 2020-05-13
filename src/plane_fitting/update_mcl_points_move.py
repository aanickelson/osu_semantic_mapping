
import rospy

from nav_msgs.msg import Path, Odometry
import tf
import csv
import numpy as np
from math import pi, cos, sin
import matplotlib
from matplotlib import pyplot as plt
import time

class OdomTransform:
    def __init__(self):
        # Store the prior message (roll, pitch, yaw), current message, and difference between them
        self.last_msg = None
        self.this_msg = None
        # Difference will be cumulative since the last time it was used, as the MCL updates and odom updates will
        # likely be out of sync.
        self.diff = [0, 0, 0]

        self.points = None
        self.t0 = time.time()

    def get_diff(self, msg):
        # Translate to euler coordinates
        comp = 0
        (_, _, yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                                              msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        # Update current odom reading
        self.this_msg = [round(msg.pose.pose.position.x, 4), round(msg.pose.pose.position.y, 4), round(yaw, 4)]
        # print(self.this_msg[0])
        # Calculate difference between them
        if self.last_msg:
            self.diff = np.add(self.diff, [self.this_msg[0] - self.last_msg[0],
                         self.this_msg[1] - self.last_msg[1],
                         self.this_msg[2] - self.last_msg[2]])
            comp = abs(self.diff[0]) + abs(self.diff[1]) + abs(self.diff[2])

        if time.time() - self.t0 > 10:
            self.translate_points()
            self.t0 = time.time()

        # Update current message to be the last message received
        self.last_msg = self.this_msg

    def translate_points(self):
        # Placeholder for testing
        points = np.random.rand(100, 4)
        for i in range(len(points)):
            points[i][1] *= 3
            points[i][0] *= 3
            points[i][2] = pi / 2

        init_points = points.copy()
        # Add a 0 to diff array as a placeholder for the probability so it can be easily subtracted
        diff = np.append(self.diff, [0])
        # diff = np.array([1, 1, pi/4, 0])
        # Loop through and translate to new position
        for i in range(len(points)):
            points[i] = np.subtract(points[i], diff)

        # Reset the difference to zero so it can start counting up again
        self.diff = [0, 0, 0]

        self.plot_it(init_points, points)

        return points

    def get_arrays(self, points):
        x = []
        y = []
        cos_yaw = []
        sin_yaw = []
        for point in points:
            x.append(point[0])
            y.append(point[1])
            cos_yaw.append(cos(point[2]))
            sin_yaw.append(sin(point[2]))
        return x, y, cos_yaw, sin_yaw

    def plot_it(self, init_points, points):
        x_0, y_0, cos_0, sin_0 = self.get_arrays(init_points)
        x_1, y_1, cos_1, sin_1 = self.get_arrays(points)
        fig, ax = plt.subplots()
        q_0 = ax.quiver(x_0, y_0, cos_0, sin_0, color='b')
        q_1 = ax.quiver(x_1, y_1, cos_1, sin_1)
        plt.show()

if __name__ == "__main__":
    odom = OdomTransform()
    rospy.init_node('update_mcl_points_py', anonymous=True)

    rospy.Subscriber("/odom", Odometry, odom.get_diff, queue_size=1)

    rospy.spin()

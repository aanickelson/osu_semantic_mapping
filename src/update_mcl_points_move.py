
import rospy

from nav_msgs.msg import Path, Odometry
import tf
import csv
import numpy as np
from math import pi

class OdomTransform:
    def __init__(self):
        # Store the prior message (roll, pitch, yaw), current message, and difference between them
        self.last_msg = [0, 0, 0]
        self.this_msg = [0, 0, 0]
        # Difference will be cumulative since the last time it was used, as the MCL updates and odom updates will
        # likely be out of sync.
        self.diff = [0, 0, 0]

    def get_diff(self, msg):
        # Translate to euler coordinates
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                                              msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        # Update current odom reading
        self.this_msg = [r, p, y]

        # Calculate difference between them
        self.diff += [self.this_msg[0] - self.last_msg[0],
                     self.this_msg[1] - self.last_msg[1],
                     self.this_msg[2] - self.last_msg[2]]

        # Update current message to be the last message received
        self.last_msg = self.this_msg

    def translate_points(self, points):
        # Add a 0 to diff array as a placeholder for the probability so it can be easily subtracted
        diff = np.array(self.diff + [0])

        # Loop through and translate to new position
        for i in range(len(points)):
            points[i] = np.subtract(points[i], diff)

        # Reset the difference to zero so it can start counting up again
        self.diff = [0, 0, 0]

        return points


if __name__ == "__main__":
    odom = OdomTransform()
    rospy.init_node('update_mcl_points_py', anonymous=True)

    # Placeholder for testing
    points = np.random.rand(100, 4)
    for i in range(len(points)):
        points[i][1] *= 5
        points[i][0] *= 5
        points[i][2] *= 2 * pi

    rospy.Subscriber("/odom", Odometry, odom.get_diff)
    print(odom.translate_points(points))
    rospy.spin()

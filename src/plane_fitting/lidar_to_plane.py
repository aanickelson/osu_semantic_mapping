
# Code copied from https://stackoverflow.com/questions/39772424/how-to-effeciently-convert-ros-pointcloud2-to-pcl-point-cloud-and-visualize-it-i


import matplotlib
# matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rospy
import pcl
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from mpl_toolkits.mplot3d import Axes3D
from plane_class import PlaneObject


class LidartoPlane:
    def __init__(self):
        self.pcl_data = None
        self.it = 0
        self.x_orig = []
        self.y_orig = []
        self.z_orig = []
        self.plane = PlaneObject()

    def ros_to_pcl(self, ros_cloud):
        """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB
            Copied from https://www.programcreek.com/python/example/99841/sensor_msgs.msg.PointCloud2
            Altered to use PointCloud instead of PointCloud_PointXYZRGB

            Args:
                ros_cloud (PointCloud2): ROS PointCloud2 message

            Returns:
                pcl.PointCloud: PCL point cloud
        """
        points_list = []

        for data in pc2.read_points(ros_cloud, skip_nans=True):
            points_list.append([data[0], data[1], data[2]])
            self.x_orig.append(data[0])
            self.y_orig.append(data[1])
            self.z_orig.append(data[2])

        self.pcl_data = pcl.PointCloud()
        self.pcl_data.from_list(points_list)

        if self.it == 10:
            self.planify()

        self.it += 1

    def planify(self):
        print('Point cloud data: ' + str(self.pcl_data.size) + ' points')
        # for i in range(0, cloud.size):
        #     print('x: ' + str(cloud[i][0]) + ', y : ' +
        #           str(cloud[i][1]) + ', z : ' + str(cloud[i][2]))

        seg = self.pcl_data.make_segmenter_normals(ksearch=50)
        seg.set_optimize_coefficients(True)
        seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(0.01)
        seg.set_normal_distance_weight(0.01)
        seg.set_max_iterations(100)
        indices, self.plane.coefficients = seg.segment()

        if len(indices) == 0:
            print('Could not estimate a planar model for the given dataset.')
            exit(0)

        print('Model coefficients: ' + str(self.plane.coefficients[0]) + ' ' + str(
            self.plane.coefficients[1]) + ' ' + str(self.plane.coefficients[2]) + ' ' + str(self.plane.coefficients[3]))

        print('Model inliers: ' + str(len(indices)))

        x_n = []
        y_n = []
        z_n = []
        for i in range(0, len(indices)):
            x_n.append(self.pcl_data[indices[i]][0])
            y_n.append(self.pcl_data[indices[i]][1])
            z_n.append(self.pcl_data[indices[i]][2])
            # print(str(indices[i]) + ', x: ' + str(cloud[indices[i]][0]) + ', y : ' +
            #       str(cloud[indices[i]][1]) + ', z : ' + str(cloud[indices[i]][2]))
            self.plane.inliers.append([self.pcl_data[indices[i]][0],
                                       self.pcl_data[indices[i]][1],
                                       self.pcl_data[indices[i]][2]])

        plot_plane(self.plane.coefficients, x_n, y_n, z_n, self.x_orig, self.y_orig, self.z_orig)


def plot_plane(coefficients, x_in, y_in, z_in, x_orig, y_orig, z_orig):
    x_range = [np.min(x_orig), np.max(x_orig)]
    y_range = [np.min(y_orig), np.max(y_orig)]
    z_range = [np.min(z_orig), np.max(z_orig)]

    num_slice = 10
    x = x_in[::num_slice]
    y = y_in[::num_slice]
    z = z_in[::num_slice]

    x_orig = x_orig[::num_slice]
    y_orig = y_orig[::num_slice]
    z_orig = z_orig[::num_slice]

    # # THIS IS A STRONG ASSUMPTION - assuming that the x and y coordinates are uniformly distributed
    # # and that every x overlaps with every y (i.e. that it's a square).
    # x_plane = np.arange(x_range[0], x_range[1], 0.01)
    # y_plane = np.arange(y_range[0], y_range[1], 0.01)
    #
    # X, Y = np.meshgrid(x, y)

    print('thinking.')

    # POINTS ---------------------------------
    fig = plt.figure()
    ax = fig.add_subplot(1, 3, 1, projection='3d')
    surf = ax.scatter(x, y, z, color='b')
    ax.set_xlim(x_range[0] - 0.5, x_range[1] + 0.5)
    ax.set_ylim(y_range[0] - 0.5, y_range[1] + 0.5)
    ax.set_zlim(z_range[0] - 0.5, z_range[1] + 0.5)

    ax2 = fig.add_subplot(1, 3, 2, projection='3d')
    surf = ax2.scatter(x_orig, y_orig, z_orig, color='g')
    ax2.set_xlim(x_range[0] - 0.5, x_range[1] + 0.5)
    ax2.set_ylim(y_range[0] - 0.5, y_range[1] + 0.5)
    ax2.set_zlim(z_range[0] - 0.5, z_range[1] + 0.5)

    # PLANE -------------------------------
    a = coefficients[0]
    b = coefficients[1]
    c = coefficients[2]
    d = coefficients[3]

    z_plot = -(a * np.asarray(x) + b * np.asarray(y) + d) / c

    ax3 = fig.add_subplot(1, 3, 3, projection='3d')
    print('thinking..')
    ax3.scatter(x, y, z_plot, c='r', marker='o')
    ax3.set_xlabel('X-axis')
    ax3.set_ylabel('Y-axis')
    ax3.set_zlabel('Z-axis')
    ax3.set_xlim(x_range[0] - 0.5, x_range[1] + 0.5)
    ax3.set_ylim(y_range[0] - 0.5, y_range[1] + 0.5)
    ax3.set_zlim(z_range[0] - 0.5, z_range[1] + 0.5)

    print('thinking...')

    plt.show()

if __name__ == "__main__":
    lidar = LidartoPlane()
    rospy.init_node('lidar_to_plane_py', anonymous=True)
    rospy.Subscriber("/head_camera/depth_downsample/points", PointCloud2, lidar.ros_to_pcl)

    rospy.spin()




# Code copied from https://stackoverflow.com/questions/39772424/how-to-effeciently-convert-ros-pointcloud2-to-pcl-point-cloud-and-visualize-it-i


import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rospy
import pcl
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from mpl_toolkits.mplot3d import Axes3D


class LidarData:
    def __init__(self):
        self.p = None
        self.it = 0

    def callback(self, data):
        pc = ros_numpy.numpify(data)

        height = pc.shape[0]
        width = pc.shape[1]
        points = np.zeros((height * width, 3), dtype=np.float32)
        points[:, 0] = np.resize(pc['x'], height * width)
        points[:, 1] = np.resize(pc['y'], height * width)
        points[:, 2] = np.resize(pc['z'], height * width)

        p = pcl.PointCloud(np.array(points, dtype=np.float32))

        x_range = [np.min(points[:, 0]), np.max(points[:, 0])]
        y_range = [np.min(points[:, 1]), np.max(points[:, 1])]
        z_range = [np.min(points[:, 2]), np.max(points[:, 2])]

        if self.it == 10:
            self.p = p
            main(self.p, x_range, y_range, z_range, points)

        self.it += 1

    def ros_to_pcl(self, ros_cloud):
        """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB
            Copied from https://www.programcreek.com/python/example/99841/sensor_msgs.msg.PointCloud2
            Altered to use PointCloud instead of PointCloud_PointXYZRGB

            Args:
                ros_cloud (PointCloud2): ROS PointCloud2 message

            Returns:
                pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud
        """
        points_list = []
        x = []
        y = []
        z = []

        for data in pc2.read_points(ros_cloud, skip_nans=True):
            # print("data", (data[0]))
            points_list.append([data[0], data[1], data[2]])  #, data[3]])
            x.append(data[0])
            y.append(data[1])
            z.append(data[2])

        pcl_data = pcl.PointCloud()
        pcl_data.from_list(points_list)

        if self.it == 10:
            self.p = pcl_data
            main(self.p, x, y, z)

        self.it += 1


def main(cloud, x, y, z):
    print('Point cloud data: ' + str(cloud.size) + ' points')
    # for i in range(0, cloud.size):
    #     print('x: ' + str(cloud[i][0]) + ', y : ' +
    #           str(cloud[i][1]) + ', z : ' + str(cloud[i][2]))

    seg = cloud.make_segmenter_normals(ksearch=50)
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.01)
    seg.set_normal_distance_weight(0.01)
    seg.set_max_iterations(100)
    indices, coefficients = seg.segment()

    if len(indices) == 0:
        print('Could not estimate a planar model for the given dataset.')
        exit(0)

    print('Model coefficients: ' + str(coefficients[0]) + ' ' + str(
        coefficients[1]) + ' ' + str(coefficients[2]) + ' ' + str(coefficients[3]))

    print('Model inliers: ' + str(len(indices)))

    x_n = []
    y_n = []
    z_n = []
    for i in range(0, len(indices)):
        x_n.append(cloud[indices[i]][0])
        y_n.append(cloud[indices[i]][1])
        z_n.append(cloud[indices[i]][2])
        # print(str(indices[i]) + ', x: ' + str(cloud[indices[i]][0]) + ', y : ' +
        #       str(cloud[indices[i]][1]) + ', z : ' + str(cloud[indices[i]][2]))

    plot_plane(coefficients, x, y, z)


def plot_plane(coefficients, x, y, z):
    x_range = [np.min(x), np.max(y)]
    y_range = [np.min(y), np.max(y)]

    # PLANE -------------------------------

    # THIS IS A STRONG ASSUMPTION - assuming that the x and y coordinates are uniformly distributed
    # and that every x overlaps with every y (i.e. that it's a square).
    x_plane = np.arange(x_range[0], x_range[1], 0.01)
    y_plane = np.arange(y_range[0], y_range[1], 0.01)

    X, Y = np.meshgrid(x, y)

    a = coefficients[0]
    b = coefficients[1]
    c = coefficients[2]
    d = coefficients[3]

    z_plot = (-a * X - b * Y + d) / c

    fig = plt.figure()
    ax = fig.add_subplot(1, 2, 1, projection='3d')

    surf = ax.scatter(x, y, z, color='b')

    # POINTS ---------------------------------
    ax2 = fig.add_subplot(1, 2, 2, projection='3d')

    # Plot the values
    ax2.scatter(X, Y, z_plot, c='r', marker='o')
    ax2.set_xlabel('X-axis')
    ax2.set_ylabel('Y-axis')
    ax2.set_zlabel('Z-axis')

    plt.show()

if __name__ == "__main__":
    lidar = LidarData()
    rospy.init_node('lidar_to_plane_py', anonymous=True)
    rospy.Subscriber("/head_camera/depth_downsample/points", PointCloud2, lidar.ros_to_pcl)

    rospy.spin()



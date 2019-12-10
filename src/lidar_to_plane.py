
# Code copied from https://stackoverflow.com/questions/39772424/how-to-effeciently-convert-ros-pointcloud2-to-pcl-point-cloud-and-visualize-it-i

import rospy
import pcl
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class LidarData:
    def __init__(self):
        self.p = None
        self.it = 0

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
            # print("data", (data))
            points_list.append([data[0], data[1], data[2]])  #, data[3]])
            x.append(data[0])
            y.append(data[1])
            z.append(data[2])

        pcl_data = pcl.PointCloud()
        pcl_data.from_list(points_list)

        if self.it == 0:
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
    # for i in range(0, len(indices)):
    #     print(str(indices[i]) + ', x: ' + str(cloud[indices[i]][0]) + ', y : ' +
    #           str(cloud[indices[i]][1]) + ', z : ' + str(cloud[indices[i]][2]))

    plot_plane(coefficients, x, y, z)


def plot_plane(coefficients, x_points, y_points, z_points):
    # PLANE -------------------------------
    x_plane = np.linspace(0, 8)
    y_plane = np.linspace(0, 8)

    # print(x_plane)
    #
    # x_plot, y_plot = np.meshgrid(x_plane, y_plane)

    a = coefficients[0]
    b = coefficients[1]
    c = coefficients[2]
    d = coefficients[3]

    z_plot = (a * x_plane + b * y_plane + d) / c

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    surf = ax.plot_surface(x_plane, y_plane, z_plot, color='b')

    # POINTS ---------------------------------
    ax2 = fig.add_subplot(111, projection='3d')

    # Plot the values
    ax2.scatter(x_points, y_points, z_points, c='r', marker='o')
    ax2.set_xlabel('X-axis')
    ax2.set_ylabel('Y-axis')
    ax2.set_zlabel('Z-axis')

    plt.show()

if __name__ == "__main__":
    lidar = LidarData()
    rospy.init_node('lidar_to_plane_py', anonymous=True)
    rospy.Subscriber("/head_camera/depth_downsample/points", PointCloud2, lidar.ros_to_pcl)

    rospy.spin()



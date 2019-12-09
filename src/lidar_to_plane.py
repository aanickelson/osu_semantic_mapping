
# Code copied from https://stackoverflow.com/questions/39772424/how-to-effeciently-convert-ros-pointcloud2-to-pcl-point-cloud-and-visualize-it-i

import rospy
import pcl
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np


class LidarData:
    def __init__(self):
        self.p = None
        self.it = 0

    def callback(self, data):
        print('is this being called')
        pc = ros_numpy.numpify(data)
        points = np.zeros((pc.shape[0],3))
        points[:, 0] = pc['x']
        points[:, 1] = pc['y']
        points[:, 2] = pc['z']
        if self.it == 0:
            self.p = pcl.PointCloud(np.array(points, dtype=np.float32))
            main(self.p)
        self.it += 1


def main(cloud):
    print('Point cloud data: ' + str(cloud.size) + ' points')
    for i in range(0, cloud.size):
        print('x: ' + str(cloud[i][0]) + ', y : ' +
              str(cloud[i][1]) + ', z : ' + str(cloud[i][2]))

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
    for i in range(0, len(indices)):
        print(str(indices[i]) + ', x: ' + str(cloud[indices[i]][0]) + ', y : ' +
              str(cloud[indices[i]][1]) + ', z : ' + str(cloud[indices[i]][2]))


if __name__ == "__main__":
    lidar = LidarData()
    rospy.init_node('lidar_to_plane_py', anonymous=True)
    rospy.Subscriber("/head_camera/depth_downsample/points", PointCloud2, lidar.callback)

    rospy.spin()



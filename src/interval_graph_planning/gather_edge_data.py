#!/usr/bin/env python

import rospy

# Brings in the .action file and messages used by the move base action
from geometry_msgs.msg import PoseWithCovarianceStamped
from hospital_graph_class import HospitalGraph

from two_rooms_v3_parameters import TwoRoomsParameters
import interval_cust


class RoboInfo:
    def __init__(self, p):
        self.current_pose = None
        self.prior_node = None
        self.current_node = None
        self.p = p
        # self.building = HospitalGraph(p.num_rooms, p.num_halls, p.extra_doors,
        #                               p.hall_door_links, p.extra_door_hall_links, p.connected_halls)

    def set_current_pose(self, msg):
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        print(self.current_pose)
        self.what_node()
        print(self.current_node)

        if self.prior_node != self.current_node:
            # TODO: Publish that the robot transitioned between nodes
            # print(self.current_node)
            pass

    def what_node(self):
        """
       Door nodes are checked first - they are within the bounds of rooms / halls so doors must supersede other areas

        Args:
            p: parameters
            loc: current x, y location to check

        Returns:
            node the robot is currently in

        """
        self.prior_node = self.current_node

        # Loop through once for doors
        for node in self.p.nodes_dict:
            if 'd' in node:
                if self.current_pose[0] in self.p.nodes_dict[node][0] and self.current_pose[1] in self.p.nodes_dict[node][1]:
                    self.current_node = node

        # Loop through a second time for all other areas
        for node in self.p.nodes_dict:
            if 'd' in node:
                continue
            else:
                if self.current_pose[0] in self.p.nodes_dict[node][0] and self.current_pose[1] in self.p.nodes_dict[node][1]:
                    self.current_node = node

        raise ValueError('Robot is out of this world')


if __name__ == "__main__":
    rospy.init_node('edge_loc_py')

    r = RoboInfo(p=TwoRoomsParameters)

    amcl_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, r.set_current_pose)

    # r.current_loc = [2, -3]
    # print(what_node(TwoRoomsParameters.nodes_dict, r.current_loc))
    #
    # r.current_loc = [-2, -2]
    # print(what_node(TwoRoomsParameters.nodes_dict, r.current_loc))
    #
    # r.current_loc = [2, 2]
    # print(what_node(TwoRoomsParameters.nodes_dict, r.current_loc))




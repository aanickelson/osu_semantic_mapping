#!/usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg import GoalStatus
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from random import uniform
from math import sqrt
from time import time
import gather_edge_data as gather
from two_rooms_v3_parameters import TwoRoomsParameters as p
from random import randint


class Fetching:
    def __init__(self, nodes_dict):
        self.prior_node = None
        self.current_position = None
        self.current_node = None
        self.nodes_dict = nodes_dict

        self.nodes = ['r0', 'r1']
        # [[Room 0 goals [x, y]], [room 1 goals [x, y]]]
        self.goals = [[[-9., -1.], [-9., -4.], [-8.5, -2.5], [-7., -1.], [-7., -4.]],
                      [[1., -1.], [1., -4.], [2.5, -2.5], [4., -1.], [4., -4.]]]

        self.prior_goal = None
        self.next_goal = None

    def set_current_pose(self, msg):
        # Get the current position of the robot and store it
        # (2, -2) is the transform between the robot's map and the actual map
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        # Update the current quadrant - will be used to select next goal
        print(self.current_position)
        # self.prior_node = self.current_node
        # self.current_node = gather.what_node(self.nodes_dict, self.current_position)

        if self.prior_node != self.current_node:
            #TODO: Publish that the robot transitioned between nodes
            pass

    def select_new_goal(self):
        choose_one = randint(0, 5)

        # Move current goal to prior goal
        self.prior_goal = self.next_goal

        # Pick a new goal
        if self.current_node == 'r0':  # If you're in room 0
            self.next_goal = self.goals[1][choose_one]  # Choose a goal from room 1
        elif self.current_node == 'r1':
            self.next_goal = self.goals[0][choose_one]  # Choose a goal from room 0
        else:
            raise ValueError("You have not reached your previous goal. Get back to work...")

    def movebase_client(self, redo=False):
        # Code originally copied from https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()

        # if not redo:
        #     # Select a new goal
        #     self.prior_goal = self.next_goal
        #     # try:
        #     self.select_new_goal()
        #     # except ValueError:  # If the robot is not in a room when it tries to get a new goal, send it to prior location
        #     #     self.next_goal = self.prior_goal
        # else:
        #     # Go back to the last point
        #     self.next_goal = self.prior_goal

        self.next_goal = [2, 2]

        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Move 0.5 meters forward along the x axis of the "map" coordinate frame
        goal.target_pose.pose.position.x = self.next_goal[0]
        goal.target_pose.pose.position.y = self.next_goal[1]
        goal.target_pose.pose.position.z = 0.0

        # No rotation of the mobile base frame w.r.t. map frame
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        wait = client.wait_for_result()

        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            # rospy.signal_shutdown("Action server not available!")
        else:
            # Result of executing the action
            return client.get_state()


if __name__ == '__main__':
    rospy.init_node('movebase_client_py')

    fetch_fetching = Fetching(p.nodes_dict)

    # Subscribe to a bunch of stuff to get a bunch of info.
    amcl_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, fetch_fetching.set_current_pose)
    # move_base_sub = rospy.Subscriber('move_base/NavfnROS/plan', Path, rowboat_robot.get_path_info)
    # odom_sub = rospy.Subscriber('odom', Odometry, rowboat_robot.get_odom_info)
    # laser_sub = rospy.Subscriber('scan', LaserScan, rowboat_robot.get_laser_info)

    iteration = 0
    total_iterations = 1

    # Redo is a flag that, if true, sends the robot back to the previous point because something went wrong
    redo = False

    try:
        while iteration < total_iterations:

            # Tried to break it in such a way that I could catch it. Turns out it's too good.
            # It either succeeds or can't reach the goal.
            fetch_fetching.final_it = False

            # if iteration == total_iterations - 2:
            #     # Out of range
            #     rowboat_robot.final_it = True

            print("##############")
            print("Iteration {}".format(iteration))
            print("##############")

            start_time = time()
            odom_path_length = 0
            result = fetch_fetching.movebase_client(redo)
            iteration += 1

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

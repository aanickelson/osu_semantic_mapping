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
import csv


# Name inspired by this artist - http://www.robotsinrowboats.com/
class RobotRowing:
    def __init__(self):
        self.current_position = None
        self.current_quadrant = None
        self.quadrants = [0, 1, 2, 3]

        # Planned paths from move_base
        self.x_paths = []
        self.y_paths = []
        self.x_current_path = None
        self.y_current_path = None

        # Metadata on paths
        self.all_paths_info = []
        self.current_path_info = []

        # Path info from odom
        self.x_odom = []
        self.y_odom = []
        self.x_current_odom = []
        self.y_current_odom = []
        self.final_it = False

        # Keeps track of lowest laser scan reading - how close did this path get to the wall at the closest distance
        self.got_real_close_there = 10000000

        self.next_goal = (-0.5, 2.0)  # Initialize to the center of the map
        self.prior_goal = (-0.5, 2.0)
        self.break_goal = (-3.16, 4.66)
        self.result_from_path = 3

        # Quadrants in x and y - quadrants for goal are smaller than for localization
        self.x_goal_positions = [[-0.5, 2], [-0.5, 2], [-3, -0.5], [-3, -0.5]]
        self.x_localize_positions = [[-0.5, 2.3], [-0.5, 2.3], [-3.5, -0.5], [-3.5, -0.5]]

        self.y_goal_positions = [[2.3, 4.5], [-0.4, 1.7], [2.3, 4.5], [-0.4, 1.7]]
        self.y_localize_positions = [[2.0, 4.8], [-0.8, 2.0], [2.0, 4.8], [-0.8, 2.0]]

    def set_current_pose(self, msg):
        # Get the current position of the robot and store it
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        # Update the current quadrant - will be used to select next goal
        for quad in self.quadrants:
            x_loc_range = self.x_localize_positions[quad]
            y_loc_range = self.y_localize_positions[quad]
            if x_loc_range[0] <= self.current_position[0] <= x_loc_range[1] and \
               y_loc_range[0] <= self.current_position[1] <= y_loc_range[1]:
                self.current_quadrant = quad

    def select_new_goal(self):
        # Robot will move from quadrant 0 -> 1 -> 2 -> 3 -> 0
        map = [1, 2, 3, 0]
        new_quad = map[self.current_quadrant]
        # if not self.final_it:
        # Uniformly select a random point in the new quadrant
        self.next_goal = (uniform(self.x_goal_positions[new_quad][0], self.x_goal_positions[new_quad][1]),
                              uniform(self.y_goal_positions[new_quad][0], self.y_goal_positions[new_quad][1]))
        # else:
        #     self.next_goal = self.break_goal
        print("Next goal chosen:", self.next_goal)

    def get_odom_info(self, msg):
        # Store the messages from odom to reconstruct path
        self.x_current_odom.append(msg.pose.pose.position.x)
        self.y_current_odom.append(msg.pose.pose.position.y)

    def get_path_info(self, msg):
        # Only store the path the first time it is published after the next goal is chosen
        if self.x_current_path is None and self.y_current_path is None:
            self.x_current_path = []
            self.y_current_path = []
            for i in range(len(msg.poses)):
                self.x_current_path.append(msg.poses[i].pose.position.x)
                self.y_current_path.append(msg.poses[i].pose.position.y)

            self.x_paths.append(self.x_current_path)
            self.y_paths.append(self.y_current_path)

    def get_laser_info(self, msg):
        # Keeps the closest point seen yet
        for point in msg.ranges:
            if point < self.got_real_close_there:
                self.got_real_close_there = point

    def movebase_client(self, redo=False):
        # Code originally copied from https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()

        if not redo:
            # Select a new goal
            self.prior_goal = self.next_goal
            self.select_new_goal()
        else:
            # Go back to the last point
            self.next_goal = self.prior_goal

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

        # Each time it gets a new goal, it resets the current path
        self.x_current_path = None
        self.y_current_path = None

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

    total_iterations = 6

    # Make a rowing robot
    rowboat_robot = RobotRowing()

    # Subscribe to a bunch of stuff to get a bunch of info.
    amcl_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, rowboat_robot.set_current_pose)
    move_base_sub = rospy.Subscriber('move_base/NavfnROS/plan', Path, rowboat_robot.get_path_info)
    odom_sub = rospy.Subscriber('odom', Odometry, rowboat_robot.get_odom_info)
    laser_sub = rospy.Subscriber('scan', LaserScan, rowboat_robot.get_laser_info)

    # For catching failures
    fail_options = [0, 1, 2, 4, 5, 6, 7, 8, 9]
    iteration = 0
    # Redo is a flag that, if true, sends the robot back to the previous point because something went wrong
    redo = False

    try:
        while iteration < total_iterations:

            # Tried to break it in such a way that I could catch it. Turns out it's too good.
            # It either succeeds or can't reach the goal.
            rowboat_robot.final_it = False

            # if iteration == total_iterations - 2:
            #     # Out of range
            #     rowboat_robot.final_it = True

            print("##############")
            print("Iteration {}".format(iteration))
            print("##############")

            start_time = time()
            odom_path_length = 0
            result = rowboat_robot.movebase_client(redo)

            if result:
                final_time = time() - start_time

                if len(rowboat_robot.x_current_odom) > 25:
                    # Saves most recent path received from odom
                    rowboat_robot.x_odom.append(rowboat_robot.x_current_odom)
                    rowboat_robot.y_odom.append(rowboat_robot.y_current_odom)

                    # Calculates the total distance traveled by the robot. May slightly underestimate, but works for this purpose
                    # TO DO - DEBUG WHY THIS IS NOT CALCULATING CORRECTLY
                    # NEVER MIND - TURNS OUT I WAS LOOKING AT TIME INSTEAD OF PATH LENGTH.
                    for i in range(len(rowboat_robot.x_current_odom) - 1):
                        x_dist = rowboat_robot.x_current_odom[i] - rowboat_robot.x_current_odom[i+1]
                        y_dist = rowboat_robot.y_current_odom[i] - rowboat_robot.y_current_odom[i+1]
                        odom_path_length += sqrt(x_dist**2 + y_dist**2)

                    # Save all path metadata
                    rowboat_robot.current_path_info = [result, final_time, odom_path_length, rowboat_robot.got_real_close_there]
                    rowboat_robot.all_paths_info.append(rowboat_robot.current_path_info)
                    print("results", rowboat_robot.current_path_info)

                # Re-set the current odom to empty arrays for the next path to be captured
                rowboat_robot.x_current_odom = []
                rowboat_robot.y_current_odom = []

                # Sometimes the next loop runs before the path is complete, this removes that information
                # This also re-runs the current iteration. It's funky, but it works.
                if len(rowboat_robot.x_paths[-1]) < 4:
                    rowboat_robot.x_paths = rowboat_robot.x_paths[0:-1]
                    rowboat_robot.y_paths = rowboat_robot.y_paths[0:-1]

                elif result != 3:
                    rowboat_robot.result_from_path = result
                    redo = True
                    iteration += 1
                    print("Robot did not execute proper path")

                else:
                    redo = False
                    iteration += 1
                    print("Arrived at:", rowboat_robot.current_position)
                    rospy.loginfo("Goal execution done!")

                # Re-initialize everything before next path is chosen
                rowboat_robot.x_current_odom = []
                rowboat_robot.y_current_odom = []
                rowboat_robot.got_real_close_there = 10000000

            else:
                # For when something goes terribly wrong
                print("Something has gone terribly wrong")
                break

    # except (KeyboardInterrupt, SystemExit):
    #     pass
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
        # rospy.signal_shutdown("IT DED")
        raise

    # # Save path info to csv
    # with open('./src/me599_proj/Results/planned_paths.csv', mode='w') as path_file:
    #     path_writer = csv.writer(path_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    #
    #     for num_paths in range(total_iterations):
    #         write_paths = [rowboat_robot.x_paths[num_paths], rowboat_robot.y_paths[num_paths]]
    #         path_writer.writerow(write_paths)
    #
    # # Save odom info to csv
    # with open('./src/me599_proj/Results/odom_paths.csv', mode='w') as odom_file:
    #     odom_writer = csv.writer(odom_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    #
    #     for num_odom in range(len(rowboat_robot.x_odom)):
    #         write_odom = [rowboat_robot.x_odom[num_odom], rowboat_robot.y_odom[num_odom]]
    #         odom_writer.writerow(write_odom)
    #
    # # Save path metadata to csv
    # with open('./src/me599_proj/Results/path_metadata.csv', mode='w') as meta_file:
    #     meta_writer = csv.writer(meta_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    #
    #     for num_meta in range(len(rowboat_robot.all_paths_info)):
    #         write_meta = [rowboat_robot.all_paths_info[num_meta]]
    #         meta_writer.writerow(write_meta)

    print("done!")

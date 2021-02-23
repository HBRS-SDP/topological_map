#! /usr/bin/env python

import rospy
import actionlib
import numpy as np
from transforms3d import euler

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from actionlib_msgs.msg import *


class TopologicalNavigator:
    def __init__(self):
        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )

        if self.checkMoveBaseClient():
            self.goal_node = MoveBaseGoal()

            self.robot_pose = Pose()

            self.robot_pose_subscriber = rospy.Subscriber(
                "amcl_pose", PoseWithCovarianceStamped, callback=self.getRobotPose
            )

    def checkMoveBaseClient(self):
        # wait for get_topology_node server
        rospy.loginfo("[Topological Navigator] Waiting for move_base action server...")

        if self.move_base_client.wait_for_server(rospy.Duration(60)):
            rospy.loginfo(
                "[Topological Navigator] Connected to move_base action server!"
            )
            return True
        else:
            rospy.logerr(
                "[Topological Navigator] Failed to connect to move_base action server..."
            )
            return False

    def executeNavigation(self, waypoint, desired_orientation):

        if self.checkPositionWithinTolerance(waypoint):
            rospy.loginfo("[Topological Navigator] Robot already at waypoint")
            return True

        rospy.loginfo("[Topological Navigator] Moving Robot...")

        self.sendMoveGoal(waypoint, desired_orientation)

        self.move_base_client.wait_for_result(rospy.Duration(60))

        if self.move_base_client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("[Topological Navigator] Robot Reach Waypoint")
            return True
        else:
            rospy.loginfo("[Topological Navigator] Robot Failed to Reach Waypoint")
            return False

    def checkPositionWithinTolerance(self, waypoint):
        x_pose_diff = waypoint.x - self.robot_pose.position.x
        y_pose_diff = waypoint.y - self.robot_pose.position.y

        if np.abs(x_pose_diff) < 0.5 and np.abs(y_pose_diff) < 0.5:
            return True

        return False

    def checkOrientationWithinTolerance(self, desired_orientation):

        robot_yaw = self.getRobotOrientation()

        orientation_diff = self.clipToPi(desired_orientation - robot_yaw)
        rospy.loginfo(
            "[Topological Navigator] Orientation Diff: %s - %s = %s",
            desired_orientation,
            robot_yaw,
            orientation_diff,
        )

        if np.abs(orientation_diff) > 1.571:
            return False

        return True

    def getRobotPose(self, pose_msg):
        self.robot_pose.position.x = pose_msg.pose.pose.position.x
        self.robot_pose.position.y = pose_msg.pose.pose.position.y
        self.robot_pose.orientation.x = pose_msg.pose.pose.orientation.x
        self.robot_pose.orientation.y = pose_msg.pose.pose.orientation.y
        self.robot_pose.orientation.z = pose_msg.pose.pose.orientation.z
        self.robot_pose.orientation.w = pose_msg.pose.pose.orientation.w

    def sendMoveGoal(self, waypoint, desired_orientation):
        self.goal_node.target_pose.header.frame_id = "/map"

        self.goal_node.target_pose.pose.position = Point(waypoint.x, waypoint.y, 0)
        quan_orientation = euler.euler2quat(0.0, 0.0, desired_orientation)

        self.goal_node.target_pose.pose.orientation.x = quan_orientation[1]
        self.goal_node.target_pose.pose.orientation.y = quan_orientation[2]
        self.goal_node.target_pose.pose.orientation.z = quan_orientation[3]
        self.goal_node.target_pose.pose.orientation.w = quan_orientation[0]

        rospy.loginfo("Sending goal location ...")
        self.move_base_client.send_goal(self.goal_node)

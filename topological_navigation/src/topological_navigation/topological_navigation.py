#! /usr/bin/env python

import rospy
import actionlib
import numpy as np
from transforms3d import euler

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist
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
            # self.robot_vel = Twist()

            self.robot_pose_subscriber = rospy.Subscriber(
                "amcl_pose", PoseWithCovarianceStamped, callback=self.getRobotPose
            )

            # self.robot_vel_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)

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

    def executeNavigation(self, waypoint):

        if self.checkPositionWithinTolerance(waypoint):
            rospy.loginfo("[Topological Navigator] Robot already at waypoint")
            return True

        # desired_orientation = self.computeOrientation(waypoint)

        # if self.checkOrientationWithinTolerance(desired_orientation) is False:
        #     rospy.loginfo("[Topological Navigator] Re-Orientating Robot.....")
        #     self.reOrientRobot(desired_orientation)
        #     rospy.loginfo("[Topological Navigator] Done Orientating Robot")

        rospy.loginfo("[Topological Navigator] Moving Robot...")

        # while self.checkPositionWithinTolerance(waypoint) is False:
        #     vel_x, ang_z = self.computeRobotVelocity(waypoint)
        #     self.moveRobot(vel_x, 0.0, ang_z)

        # self.stopRobot()

        self.sendMoveGoal(waypoint)

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

        if np.abs(orientation_diff) > 0.2:
            return False

        return True

    def computeOrientation(self, waypoint):

        return np.arctan2(
            (waypoint.y - self.robot_pose.position.y),
            (waypoint.x - self.robot_pose.position.x),
        )

    def computeRobotVelocity(self, waypoint):
        s = 0.5  # m/s (constant speed)

        desired_orientation = self.computeOrientation(waypoint)

        if self.checkOrientationWithinTolerance(desired_orientation) is False:
            if self.getRobotOrientation() > desired_orientation:
                ang_z = -0.25
            else:
                ang_z = 0.25
        else:
            ang_z = 0

        return s, ang_z

    def getRobotPose(self, pose_msg):
        self.robot_pose.position.x = pose_msg.pose.pose.position.x
        self.robot_pose.position.y = pose_msg.pose.pose.position.y
        self.robot_pose.orientation.x = pose_msg.pose.pose.orientation.x
        self.robot_pose.orientation.y = pose_msg.pose.pose.orientation.y
        self.robot_pose.orientation.z = pose_msg.pose.pose.orientation.z
        self.robot_pose.orientation.w = pose_msg.pose.pose.orientation.w

    def reOrientRobot(self, desired_orientation):
        while self.checkOrientationWithinTolerance(desired_orientation) is False:
            if self.getRobotOrientation() > desired_orientation:
                self.moveRobot(0, 0, -0.5)
            else:
                self.moveRobot(0, 0, 0.5)

    def moveRobot(self, vel_x, vel_y, ang_z):
        self.robot_vel.linear.x = vel_x
        self.robot_vel.linear.y = vel_y
        self.robot_vel.linear.z = 0.0
        self.robot_vel.angular.x = 0.0
        self.robot_vel.angular.y = 0.0
        self.robot_vel.angular.z = ang_z

        self.robot_vel_publisher.publish(self.robot_vel)

    def stopRobot(self):
        self.robot_vel.linear.x = 0.0
        self.robot_vel.linear.y = 0.0
        self.robot_vel.linear.z = 0.0
        self.robot_vel.angular.x = 0.0
        self.robot_vel.angular.y = 0.0
        self.robot_vel.angular.z = 0.0

        self.robot_vel_publisher.publish(self.robot_vel)

    def getRobotOrientation(self):
        orientation = [
            self.robot_pose.orientation.w,
            self.robot_pose.orientation.x,
            self.robot_pose.orientation.y,
            self.robot_pose.orientation.z,
        ]

        roll, pitch, yaw = euler.quat2euler(orientation)
        return yaw

    def clipToPi(self, angle):
        angle = np.fmod(angle, 2 * np.pi)

        if angle >= np.pi:
            angle -= 2 * np.pi
        elif angle <= -np.pi:
            angle += 2 * np.pi

        return angle

    def sendMoveGoal(self, waypoint):
        self.goal_node.target_pose.header.frame_id = "/map"

        self.goal_node.target_pose.pose.position = Point(waypoint.x, waypoint.y, 0)
        self.goal_node.target_pose.pose.orientation.x = 0.0
        self.goal_node.target_pose.pose.orientation.y = 0.0
        self.goal_node.target_pose.pose.orientation.z = 0.0
        self.goal_node.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Sending goal location ...")
        self.move_base_client.send_goal(self.goal_node)

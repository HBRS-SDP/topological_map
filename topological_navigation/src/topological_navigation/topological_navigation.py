#! /usr/bin/env python

import rospy
import numpy as np
from transforms3d import euler

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist


class TopologicalNavigator:
    def __init__(self):
        self.robot_pose = Pose()
        self.robot_vel = Twist()

        self.robot_pose_subscriber = rospy.Subscriber(
            "amcl_pose", PoseWithCovarianceStamped, callback=self.getRobotPose
        )

        self.robot_vel_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    def executeNavigation(self, waypoint):

        if self.checkPositionWithinTolerance(waypoint):
            rospy.loginfo("[Topological Navigator] Robot already at waypoint")
            return True

        desired_orientation = self.computeOrientation(waypoint)

        if self.checkOrientationWithinTolerance(desired_orientation) is False:
            rospy.loginfo("[Topological Navigator] Re-Orientating Robot.....")
            self.reOrientRobot(desired_orientation)
            rospy.loginfo("[Topological Navigator] Done Orientating Robot")

        rospy.loginfo("[Topological Navigator] Moving Robot...")

        while self.checkPositionWithinTolerance(waypoint) is False:
            vel_x, ang_z = self.computeRobotVelocity(desired_orientation)
            self.moveRobot(vel_x, 0.0, ang_z)
            rospy.sleep(rospy.Duration(1))

        self.stopRobot()

        rospy.loginfo("[Topological Navigator] Robot Reach Waypoint")

    def checkPositionWithinTolerance(self, waypoint):
        # rospy.loginfo(
        #     "[Topological Navigator] Desired position: %s, %s", waypoint.x, waypoint.y
        # )

        # rospy.loginfo(
        #     "[Topological Navigator] Robot's position: %s, %s",
        #     self.robot_pose.position.x,
        #     self.robot_pose.position.y,
        # )

        x_pose_diff = waypoint.x - self.robot_pose.position.x
        y_pose_diff = waypoint.y - self.robot_pose.position.y

        if np.abs(x_pose_diff) < 0.75 and np.abs(y_pose_diff) < 0.75:
            return True

        return False

    def checkOrientationWithinTolerance(self, desired_orientation):

        robot_yaw = self.getRobotOrientation()
        # rospy.loginfo("[Topological Navigator] Robot's orientation: %s", robot_yaw)

        orientation_diff = self.clipToPi(desired_orientation - robot_yaw)

        if np.abs(orientation_diff) > 0.2:
            return False

        return True

    def computeOrientation(self, waypoint):

        return np.arctan2(
            (waypoint.y - self.robot_pose.position.y),
            (waypoint.x - self.robot_pose.position.x),
        )

    def computeRobotVelocity(self, desired_orientation):
        s = 0.5  # m/s (constant speed)
        # x_diff = waypoint.x - self.robot_pose.position.x
        # y_diff = waypoint.y - self.robot_pose.position.y

        # d = np.sqrt(x_diff ** 2 + y_diff ** 2)
        # rospy.loginfo("[Topological Navigator] Distance to waypoint: %s", d)

        # vel_x = (s / d) * x_diff
        # vel_y = (s / d) * y_diff
        # rospy.loginfo("[Topological Navigator] vel_x: %s, vel_y: %s", vel_x, vel_y)
        if (self.getRobotOrientation() - desired_orientation) > 0.2:
            if self.getRobotOrientation() > desired_orientation:
                ang_z = -1
            else:
                ang_z = 1
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
                self.moveRobot(0, 0, -1)
            else:
                self.moveRobot(0, 0, 1)

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

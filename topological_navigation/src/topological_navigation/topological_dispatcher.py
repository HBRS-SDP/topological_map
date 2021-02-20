#! /usr/bin/env python

import rospy
import actionlib
import numpy as np
from transforms3d import euler

from topological_navigation.topological_navigation import TopologicalNavigator
from visualization_msgs.msg import Marker
from ropod_ros_msgs.msg import GetTopologyNodeAction, GetTopologyNodeGoal
from ropod_ros_msgs.msg import GoToAction, GoToResult


class TopologicalDispatcher:
    def __init__(self):
        self.get_topology_node_client = actionlib.SimpleActionClient(
            "/get_topology_node", GetTopologyNodeAction
        )
        if self.checkNodeClient():
            self.goto_action_server = actionlib.SimpleActionServer(
                "goto", GoToAction, execute_cb=self.goToActionExecute, auto_start=False
            )
            self.goto_action_server.start()

            self.navigator = TopologicalNavigator()

            self.marker_pub = rospy.Publisher(
                "napoleon_navigation/napoleon_driving/ropodpoints",
                Marker,
                queue_size=10,
            )

            self.marker = Marker()
            self.marker.id = 0

            self.get_topology_goal = GetTopologyNodeGoal()

            self.goto_action_result = GoToResult()

    def checkNodeClient(self):
        # wait for get_topology_node server
        rospy.loginfo(
            "[Topological Dispatcher] Waiting for get_topology_node server..."
        )

        if self.get_topology_node_client.wait_for_server(rospy.Duration(5)):
            rospy.loginfo(
                "[Topological Dispatcher] Connected to get_topology_node server!"
            )
            return True
        else:
            rospy.logerr(
                "[Topological Dispatcher] Failed to connect to get_topology_node server..."
            )
            return False

    def goToActionExecute(self, goal):
        rospy.loginfo(
            "[Topological Dispatcher] GOTO Action Received. Executing action...."
        )

        waypoints = goal.action.areas
        rospy.loginfo(
            "[Topological Dispatcher] GOTO Action received has %s waypoints.",
            len(waypoints),
        )

        for n in range(0, len(waypoints)):
            rospy.loginfo(
                "[Topological Dispatcher] Moving robot to: %s", waypoints[n].name,
            )

            waypoint_pose = self.getTopologyNode(waypoints[n]).position
            if n < (len(waypoints) - 1):
                desired_orientation = self.computeOrientation(
                    waypoint_pose, self.getTopologyNode(waypoints[n + 1]).position
                )
            else:
                desired_orientation = self.computeOrientation(
                    self.getTopologyNode(waypoints[n - 1]).position, waypoint_pose
                )

            self.visualizeWaypoint(waypoint_pose)

            result = self.navigator.executeNavigation(
                waypoint_pose, desired_orientation
            )

            if result is False:
                rospy.loginfo(
                    "[Topological Dispatcher] Robot couldn't reach %s",
                    waypoints[n].name,
                )
                self.goto_action_result.success = False
                self.goto_action_server.set_succeeded(self.goto_action_result)
                return

            rospy.loginfo(
                "[Topological Dispatcher] Robot reached %s", waypoints[n].name,
            )

        rospy.loginfo("[Topological Dispatcher] GOTO Action Complete")
        self.goto_action_result.success = True
        self.goto_action_server.set_succeeded(self.goto_action_result)

    def getTopologyNode(self, area):
        rospy.loginfo("[Topological Dispatcher] Sending Node Postion Request")

        self.get_topology_goal.id = int(area.id)
        self.get_topology_goal.ref = area.name.split("_")[-1]
        self.get_topology_goal.type = area.type

        self.get_topology_node_client.send_goal(self.get_topology_goal)
        rospy.loginfo("[Topological Dispatcher] Node Postion Request Sent!")

        if self.get_topology_node_client.wait_for_result(rospy.Duration(5)):
            rospy.loginfo("[Topological Dispatcher] Node Postion Received!")
            return self.get_topology_node_client.get_result()

        else:
            return None

    def computeOrientation(self, waypoint1, waypoint2):

        return np.arctan2((waypoint2.y - waypoint1.y), (waypoint2.x - waypoint1.x),)

    def visualizeWaypoint(self, waypoint):
        self.marker.header.frame_id = "/map"
        self.marker.id = self.marker.id + 1
        self.marker.type = 1
        self.marker.action = 0

        self.marker.pose.position.x = waypoint.x
        self.marker.pose.position.y = waypoint.y

        self.marker.pose.orientation.w = 1.0

        self.marker.scale.x = 0.25
        self.marker.scale.y = 0.25
        self.marker.scale.z = 0.01

        self.marker.color.r = 0.0
        self.marker.color.b = 0.0
        self.marker.color.g = 1.0
        self.marker.color.a = 1.0

        self.marker_pub.publish(self.marker)

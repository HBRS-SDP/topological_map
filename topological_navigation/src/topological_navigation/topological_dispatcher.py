#! /usr/bin/env python

import rospy
import actionlib
import numpy as np

from ropod_ros_msgs.msg import GetTopologyNodeAction, GetTopologyNodeGoal
from ropod_ros_msgs.msg import GoToAction, GoToResult


class TopologicalDispatcher:
    def __init__(self):

        self.get_topology_node_client = actionlib.SimpleActionClient(
            "/get_topology_node", GetTopologyNodeAction
        )
        if self.check_for_client():
            self.goto_action_server = actionlib.SimpleActionServer(
                "goto", GoToAction, execute_cb=self.goToActionExecute, auto_start=False
            )
            self.goto_action_server.start()

            self.get_topology_goal = GetTopologyNodeGoal()

            self.goto_action_result = GoToResult()

    def check_for_client(self):
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
            "[Topological Dispatcher] GOTO Action received has %s waypoints",
            len(waypoints),
        )

        if len(waypoints) > 1:
            desired_orientation = self.compute_orientation(waypoints[0], waypoints[1])

            if desired_orientation is not None:
                rospy.loginfo(
                    "[Topological Dispatcher] Desired starting orientation for robot is: %s",
                    desired_orientation,
                )
                rospy.loginfo("[Topological Dispatcher] GOTO Action Complete")
                self.goto_action_result.success = True
                self.goto_action_server.set_succeeded(self.goto_action_result)
            else:
                rospy.loginfo("[Topological Dispatcher] GOTO Action Failed!")
                self.goto_action_result.success = False
                self.goto_action_server.set_succeeded(self.goto_action_result)
        else:
            rospy.loginfo("[Topological Dispatcher] GOTO Action Complete")
            self.goto_action_result.success = True
            self.goto_action_server.set_succeeded(self.goto_action_result)

    def compute_orientation(self, area1, area2):
        area1_position = self.send_get_topology_goal(area1).position
        rospy.loginfo(
            "area1 position recieved: %s, %s", area1_position.x, area1_position.y
        )
        area2_position = self.send_get_topology_goal(area2).position
        rospy.loginfo(
            "area2 position recieved: %s, %s", area2_position.x, area2_position.y
        )

        if None in (area1_position, area2_position):
            rospy.loginfo("[Topological Dispatcher] Error receiving node position")
            return None
        else:
            return np.arctan2(
                (area2_position.y - area1_position.y),
                (area2_position.x - area1_position.x),
            )

    def send_get_topology_goal(self, area):
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

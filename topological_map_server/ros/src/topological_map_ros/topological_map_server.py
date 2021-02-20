import rospy
import actionlib
import networkx as nx

from topological_map_server.common.map_utils.map_loader import load_yaml_file
from ropod_ros_msgs.msg import GetTopologyNodeAction, GetTopologyNodeResult
from ropod_ros_msgs.msg import Position

MAP_BASE_PATH = "topological_map_server.maps."


class TopologicalMapServer:
    def __init__(self, map_name="brsu-full"):

        self.map_module = MAP_BASE_PATH + map_name
        self.map_graph = nx.node_link_graph(
            load_yaml_file(self.map_module, "topology.yaml")
        )

        self.get_topology_node_server = actionlib.SimpleActionServer(
            "get_topology_node",
            GetTopologyNodeAction,
            execute_cb=self.node_position_cb,
            auto_start=False,
        )

        self.get_topology_node_server.start()

        self._result = GetTopologyNodeResult()

    def node_position_cb(self, node):

        self._result.position = Position()

        if self.map_graph.nodes(data=True)[node.id]["label"] == node.ref:
            node_position = self.map_graph.nodes(data=True)[node.id]["pose"]

            self._result.position.x = node_position[0]
            self._result.position.y = node_position[1]
            self._result.position.id = node.id

            self.get_topology_node_server.set_succeeded(self._result)
        else:
            self.get_topology_node_server.set_aborted(self._result)

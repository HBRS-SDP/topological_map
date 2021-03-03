## Topological Navigation
High level dispatcher/navigation component responsible for receiving GOTO actions and sending move base goals for simulation of FMS tasks.

#### Action servers provided
* ROPOD GOTO

#### Action servers required
* Move base Goal
* Get Topology Node (provided by Topological Map ROS)

#### Topics subscribed
* AMCL pose

#### Run
```
roslaunch topological_navigation topological_dispatcher.launch
```

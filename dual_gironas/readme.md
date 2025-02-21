# Multidocking Docking Package

This package  contains the launch for dual Girona AUV.

## Package Usage

* To run the scenario with three robots and Docking stations: 
```
roslaunch duel_gironas tri_robot.launch 
```
* To start the action node:
```
rosrun duel_gironas multidocking_node.py
```
* To start the Dock/Undock action use the msgs: 
```
multidocking/DockingActionGoal
```
Specify the target robot (robotA, robotB, robotC) and the action (dock, undock): 
```
rostopic pub /docking_action_robotB/goal multidocking/DockingActionGoal
"header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  action: 'dock'" --once
```
* To print the feedback or Result:
```  
rostopic echo /docking_action_robotA/result
rostopic echo /docking_action_robotA/feedback
```
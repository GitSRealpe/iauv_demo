# iauv_demo

WIP for a basic demo on AUV basic operations simulation, based initially on Stonefish and COLA2 architecture

Required packages:

- girona1000_description
- cola2 framework
- [girona_utils](https://github.com/GitSRealpe/girona_utils)

### robot_description and simulation bringup
To visualize only the description of the robot you can execute the following command. Useful for checking sensor tranforms or the general kinematic chain of the robot.
``roslaunch iauv_description robot_viz_only.launch``

To run the actual simulation execute:
``roslaunch iauv_description single_robot.launch``

### iauv_motion_planner
Currently only lawnmower and circular inspection trajectories are implemented.

<table><thead>
  <tr>
    <td><img src="/media/lawnmower.png" width="300"></td>
    <td><img src="/media/circular.png" width="300"></td>
  </tr></thead>
</table>

To start the main node execute: 
``roslaunch iauv_motion_planner planner_node.launch``

The path generation is requested through the service [``/iauv_motion_planner/getPath``](https://gitsrealpe.github.io/iauv_demo/doc/html/srv/GetPath.html).

An example of building the request and calling the service to generate a lawnmower path is as follows:
```python
import rospy
from iauv_motion_planner.srv import GetPath, GetPathRequest
from iauv_motion_planner.msg import PlannerParam

service_proxy = rospy.ServiceProxy("/iauv_motion_planner/getPath", GetPath)
# create request
req = GetPathRequest()
req.header.frame_id = "world_ned"               # fixed frame
req.planner = GetPathRequest.SCANNER            # lawnmower type
param = PlannerParam()                          # parameters specific for this type of trajectory
param.key = "width"
param.value = "7"
req.params.append(param)
param = PlannerParam()
param.key = "length"
param.value = "5"
req.params.append(param)
# start position of the robot
req.start.position.x = -5
req.start.position.y = -5
req.start.position.z = 5
req.start.orientation.w = 1
# goal or center of request
req.goal.position.x = 0
req.goal.position.y = 0
req.goal.position.z = 3
req.goal.orientation.w = 1
# Call the service
response = service_proxy(req)
rospy.loginfo(f"Service response: {response}")  # Log the response
```

The complete example is found at [``request_path.py``](https://github.com/GitSRealpe/iauv_demo/blob/main/iauv_motion_planner/scripts/request_path.py) and can be executed by issuing ``rosrun iauv_motion_planner request_path.py`` while the main node is running.

### Execute a trajectory
To make the robot in simulation follow the requested path, a pursuit controller Action service from the [``girona_utils``](https://github.com/GitSRealpe/girona_utils) package can be used.

Download, compile the package and launch the main node to bringup the action service 
``roslaunch girona_utils auv_pose_controller.launch robot:=girona1000``

With the path follower main node running you can issue follow path request using a SimpleActionClient as exposed in the [``follow_path.py``](https://github.com/GitSRealpe/iauv_demo/blob/main/iauv_motion_planner/scripts/follow_path.py) example.

### rviz_cloud_annotation
This is a *fork* from [RMonica/rviz_cloud_annotation](https://github.com/RMonica/rviz_cloud_annotation) tool, with a small modification to publish the individual annotated point clouds with their label.

Labeled pointclouds are published on the ``/labeled_clouds`` topic, as a [``rviz_cloud_annotation/LabeledPointCloud2Array``](https://gitsrealpe.github.io/iauv_demo/rviz_cloud_annotation/doc/html/msg/LabeledPointCloud2Array.html).
Each message of this array is a [``LabeledPointCloud2``](https://gitsrealpe.github.io/iauv_demo/rviz_cloud_annotation/doc/html/msg/LabeledPointCloud2.html) which is compossed of:
- The PointCloud2 msg of a labeled pointcloud with the tool
- A label id, corresponding to the given number to the label by the tool
- A label name string chosen in the RViz labeling tool.

**Note**: Each label needs to have a ***name*** for a labeled pointcloud to be published on this topic 

<table><thead>
  <tr>
    <td><img src="/media/pcd_label_example.jpeg" width="600"></td>
  </tr></thead>
</table>
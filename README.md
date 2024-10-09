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

The path generation is requested through the service [``/iauv_motion_planner/getPath``](https://gitsrealpe.github.io/iauv_demo/iauv_motion_planner/doc/html/srv/GetPath.html).
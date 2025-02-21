#!/usr/bin/env python3  
from utils.ds_control import DSController
from utils.dock_undock_action import DockingServer

import rospy
from rospkg import RosPack


def main():
    # 
    DS_ALPHA = DSController(ds_name="ds_alpha")
    DS_BETA = DSController(ds_name="ds_beta")
    DS_GAMMA = DSController(ds_name="ds_charlie")
    

    # Replicate this with the robot name and the needed DS_NAME to create the dock undock action
    # For multidocking multidocking_sim.launch
    # server = DockingServer( auv_name="girona500",ds_name="ds_beta")   
    
    # For Multirobot_description tri_robot.launch
    server = DockingServer( auv_name="robotA",ds_name="ds_charlie")   
    server = DockingServer( auv_name="robotB",ds_name="ds_alpha")   
    server = DockingServer( auv_name="robotC",ds_name="ds_beta")   
 




if __name__=='__main__':
    rospy.init_node('multidocking_node', anonymous=True)
    main()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

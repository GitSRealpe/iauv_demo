#!/usr/bin/env python

import rospy
from iauv_motion_planner.srv import GetPath, GetPathRequest
from iauv_motion_planner.msg import PlannerParam


def call_service():
    # Initialize the ROS node
    rospy.init_node("service_caller_node")

    # Create a service proxy
    rospy.wait_for_service(
        "/iauv_motion_planner/getPath"
    )  # Wait until the service is available
    try:
        service_proxy = rospy.ServiceProxy(
            "/iauv_motion_planner/getPath", GetPath
        )  # Replace with your actual service type

        req = GetPathRequest()
        req.header.frame_id = "world_ned"

        req.planner = GetPathRequest.SIMPLE

        # req.planner = GetPathRequest.CIRCULAR
        # param = PlannerParam()
        # param.key = "radius"
        # param.value = "4"
        # req.params.append(param)

        # req.planner = GetPathRequest.SCANNER
        # param = PlannerParam()
        # param.key = "segment_length"
        # param.value = "0.3"
        # req.params.append(param)
        # param = PlannerParam()
        # param.key = "width"
        # param.value = "7"
        # req.params.append(param)
        # param = PlannerParam()
        # param.key = "length"
        # param.value = "5"
        # req.params.append(param)

        req.start.position.x = -10
        req.start.position.y = -2
        req.start.position.z = 8
        req.start.orientation.z = 0.707
        req.start.orientation.w = 0.707
        # goal or center of request
        req.goal.position.x = -10
        req.goal.position.y = 5 - 2
        req.goal.position.z = 5.20
        # req.goal.orientation.w = 0.707
        # req.goal.orientation.z = 0.707
        # req.goal.orientation.w = 0.924
        # req.goal.orientation.z = 0.383
        req.goal.orientation.w = 0.707
        req.goal.orientation.z = -0.707

        # Call the service
        response = service_proxy(req)
        rospy.loginfo(f"Service response: {response}")  # Log the response

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


if __name__ == "__main__":
    call_service()

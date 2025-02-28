import rospy
import actionlib
from girona_utils.msg import PursuitAction, PursuitGoal
from nav_msgs.msg import Path


def follow_path_client():
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient("robotA/pursuit_controller", PursuitAction)

    # Waits until the action server has started up and started
    # listening for goals.
    print("before the wait")
    client.wait_for_server()
    print("after the wait")

    # Creates a goal to send to the action server.
    goal = PursuitGoal()

    goal.path = rospy.wait_for_message("robotA/motion_planner/path", Path)
    goal.radius = 0.3

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()


if __name__ == "__main__":
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node("ation_path_test", anonymous=True)
        result = follow_path_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")

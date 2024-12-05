import rospy, rospkg
import sys
from statemachine import StateMachine, State

from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Pose, PoseStamped
from std_srvs.srv import Trigger
from iauv_kinematic_control.msg import TaskState
from iauv_kinematic_control.srv import SwitchTasks, SwitchTasksRequest
import numpy as np
from iauv_manipulation.srv import Motion, MotionRequest
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

import tf2_ros
import tf.transformations as tf
import copy


class IAUVIntervention(StateMachine):

    # states
    start = State(initial=True)

    auv_off_screen = State()
    auv_at_sphere = State()
    unfolded = State()
    arm_touching = State()

    # actions
    move_auv = start.to(auv_off_screen) | auv_off_screen.to(auv_at_sphere)
    move_arm = auv_at_sphere.to(unfolded) | unfolded.to(arm_touching)

    def __init__(self):
        super().__init__()

        self.auvTarget = rospy.Publisher(
            "/tp_controller/tasks/auv_configuration/target",
            PoseStamped,
            queue_size=10,
        )
        self.armTarget = rospy.Publisher(
            "/tp_controller/tasks/bravo_ee_configuration/target",
            PoseStamped,
            queue_size=10,
        )

    def on_enter_state(self, event, state):
        print(f"Now at state '{state.id}'")

    def on_exit_state(self, event, state):
        print(f"Exiting '{state.id}' state from '{event}' event.")

    def check_error(self, pos_tol, ori_tol, task: String, check_z_rate):
        pos_error = 10
        ori_error = 10
        tskstt = TaskState()
        z_old = 0
        dt = 0.1
        self.z_offset = 0.0
        counter = 0

        while pos_error > pos_tol or ori_error > ori_tol:
            tskstt: TaskState = rospy.wait_for_message(
                "/tp_controller/tasks/" + task + "/state", TaskState
            )
            pos_error = np.linalg.norm(
                np.array([tskstt.error[0], tskstt.error[1], tskstt.error[2]])
            )
            ori_error = np.linalg.norm(
                np.array([tskstt.error[3], tskstt.error[4], tskstt.error[5]])
            )
            print("ee_goal_pos_error: ", pos_error)
            print("ee_goal_ori_error: ", ori_error)
            # check cahnge of z, if doenst change then break

            if check_z_rate:
                z_rate = (tskstt.error[2] - z_old) / dt
                z_old = tskstt.error[2]
                print("rate of z:", z_rate)

                if np.fabs(z_rate) < 0.01:
                    counter += 1
                else:
                    counter = 0

                if counter == 10:
                    self.z_offset = tskstt.error[2]
                    print("z stopped changing")
                    break

            rospy.sleep(dt)

        print("achieved")

    def on_enter_auv_off_screen(self):

        rospy.wait_for_service("/girona1000/bravo/controller_manager/switch_controller")
        mtnSrv = rospy.ServiceProxy(
            "/girona1000/bravo/controller_manager/switch_controller", SwitchController
        )
        req = SwitchControllerRequest()
        req.stop_controllers = ["joint_velocity_controller"]
        req.start_controllers = ["joint_trajectory_controller"]
        req.strictness = SwitchControllerRequest.STRICT
        req.timeout = 1.0
        mtnSrv(req)
        rospy.sleep(2)

        rospy.wait_for_service("/arm_motion_srv")
        mtnSrv = rospy.ServiceProxy("/arm_motion_srv", Motion)
        req = MotionRequest()
        req.request = "fold"
        mtnSrv(req)
        rospy.sleep(5)

        print("done folding")

        rospy.wait_for_service("/tp_controller/switch_tasks")
        tskSrv = rospy.ServiceProxy("/tp_controller/switch_tasks", SwitchTasks)
        req = SwitchTasksRequest()
        req.disable_tasks = ["bravo_ee_configuration"]
        req.enable_tasks = ["auv_configuration"]
        tskSrv(req)
        rospy.sleep(2)

        auvPose = PoseStamped()
        auvPose.header.stamp = rospy.Time.now()
        auvPose.header.frame_id = "world_ned"
        auvPose.pose.position.x = -10.0
        auvPose.pose.position.y = 4.5
        auvPose.pose.position.z = 8.0
        auvPose.pose.orientation.w = 1

        self.auvTarget.publish(auvPose)

        self.check_error(0.1, 0.1, "auv_configuration", False)

        print("done")
        # go sphere
        self.send("move_auv")

    def on_enter_auv_at_sphere(self):
        # cancel girona utils

        auvPose = PoseStamped()
        auvPose.header.stamp = rospy.Time.now()
        auvPose.header.frame_id = "world_ned"
        auvPose.pose.position.x = -1.6
        auvPose.pose.position.y = 4.5
        auvPose.pose.position.z = 8.0
        auvPose.pose.orientation.w = 1

        self.auvTarget.publish(auvPose)

        self.check_error(0.1, 0.1, "auv_configuration", False)

        print("done")
        # go unfold
        self.send("move_arm")

    def on_enter_unfolded(self):
        print("unfolding")

        rospy.wait_for_service("/girona1000/bravo/controller_manager/switch_controller")
        mtnSrv = rospy.ServiceProxy(
            "/girona1000/bravo/controller_manager/switch_controller", SwitchController
        )
        req = SwitchControllerRequest()
        req.stop_controllers = ["joint_velocity_controller"]
        req.start_controllers = ["joint_trajectory_controller"]
        req.strictness = SwitchControllerRequest.STRICT
        req.timeout = 1.0
        mtnSrv(req)
        rospy.sleep(2)

        rospy.wait_for_service("/arm_motion_srv")
        mtnSrv = rospy.ServiceProxy("/arm_motion_srv", Motion)
        req = MotionRequest()
        req.request = "unfold"
        mtnSrv(req)
        rospy.sleep(8)

        print("done unfolding")
        self.send("move_arm")

    def on_enter_arm_touching(self):

        rospy.wait_for_service("/girona1000/bravo/controller_manager/switch_controller")
        mtnSrv = rospy.ServiceProxy(
            "/girona1000/bravo/controller_manager/switch_controller", SwitchController
        )
        req = SwitchControllerRequest()
        req.stop_controllers = ["joint_trajectory_controller"]
        req.start_controllers = ["joint_velocity_controller"]
        req.strictness = SwitchControllerRequest.STRICT
        req.timeout = 1.0
        mtnSrv(req)
        rospy.sleep(2)

        rospy.wait_for_service("/tp_controller/switch_tasks")
        tskSrv = rospy.ServiceProxy("/tp_controller/switch_tasks", SwitchTasks)
        req = SwitchTasksRequest()
        req.disable_tasks = []
        req.enable_tasks = ["bravo_ee_configuration"]
        tskSrv(req)
        rospy.sleep(2)

        eePose = PoseStamped()
        eePose.header.stamp = rospy.Time.now()
        eePose.header.frame_id = "world_ned"
        eePose.pose.position.x = -0.53
        eePose.pose.position.y = 5.0
        eePose.pose.position.z = 8.0
        eePose.pose.orientation.x = 0.5
        eePose.pose.orientation.y = 0.5
        eePose.pose.orientation.z = 0.5
        eePose.pose.orientation.w = 0.5
        self.armTarget.publish(eePose)

        self.check_error(0.015, 0.05, "bravo_ee_configuration", False)


if __name__ == "__main__":
    rospy.init_node("video_sequence_sm")

    sm = IAUVIntervention()

    rospack = rospkg.RosPack()
    sm._graph().write_png(
        rospack.get_path("iauv_manipulation") + "/scripts/video_sequence.png"
    )

    # print(sm._graph().to_string())
    sm.send("move_auv")

    rospy.spin()

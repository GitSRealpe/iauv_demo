
import rospy
import actionlib
import numpy as np
import tf
import os

from dual_gironas.msg import DockingAction, DockingFeedback, DockingResult  
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from statemachine import StateMachine, State
from geometry_msgs.msg import PoseWithCovarianceStamped 
from cola2_msgs.msg import BodyForceReq
from cola2_msgs.msg import WorldWaypointReq, GoalDescriptor
from nav_msgs.msg import Odometry


class DockingServer(StateMachine):
    initial_pose = State(initial=True)
    homing = State()
    entering_ds = State()
    in_ds = State()
    leave_ds = State()

    homing_menouver = (initial_pose.to(homing)
                    | homing.to(entering_ds))
    
    docking_menouver = (homing.to(entering_ds)
                    | entering_ds.to(in_ds) ) 
    
    rehome = (entering_ds.to(homing)| in_ds.to(homing))

    undock = (in_ds.to(leave_ds)| 
              leave_ds.to(initial_pose))

    def __init__(self, auv_name, ds_name):
        """
        Initializes the ROS1 Action Server.
        """
        self.auv_name = auv_name
        self.ds_name = ds_name
        self.ds_position_recieved = False
        self.yaw_axis_disable = True
        self.dock_attempt = 0
        # Define the action name
        self.action_name = f"docking_action_{self.auv_name}"
        # Create an Action Server
        self.server = actionlib.SimpleActionServer(
            self.action_name,
            DockingAction,  # Replace with your action definition
            execute_cb=self.execute_cb,
            auto_start=False
        )
        # finding the DS position
        self.update_ds_position()
        # Defining Variables
        self.new_coordinates = np.array([0.0,0.0,0.0,0.0, 0.2,0.2,0.2,0.1]) #x,y,z,yaw and tolarences set to 0.03
        self.nav_ned_T_b = None
        # subscribe
        self.b_T_ds_markers = rospy.Subscriber(f'/{self.ds_name}/body_ds_pose',PoseWithCovarianceStamped,self.b_T_ds_msg, queue_size=1)
        self.waypoint_display = rospy.Publisher(f'/{self.ds_name}/waypoint_display',PoseWithCovarianceStamped,queue_size=1)
        self.robot_pose = rospy.Subscriber(f"/{self.auv_name}/navigator/odometry",Odometry,self.AUV_odometry)

        # publish
        self.move_force = rospy.Publisher(f'/{self.auv_name}/controller/body_force_req', BodyForceReq, queue_size=1)
        self.req_pub = rospy.Publisher(f"/{self.auv_name}/controller/world_waypoint_req", WorldWaypointReq, queue_size=1)

        # Feedback and result messages
        self.feedback = DockingFeedback()
        self.result = DockingResult()

        super().__init__()
        self._graph().write_png(os.getcwd() + "/dock_action.png")

        # Start the Action Server
        self.server.start()
        self.rate = rospy.Rate(5)
        rospy.loginfo(f"Action server '{self.action_name}' started.")

    def update_ds_position(self): 
        # Retriving the DOCKING Stations position 
        self.ds_north = rospy.get_param('/stonefish_simulator/ds_north', float ) 
        self.ds_east = rospy.get_param('/stonefish_simulator/ds_east', float )  # this constant is from the DS design
        self.ds_down = rospy.get_param('/stonefish_simulator/ds_down', float) - 1.28 # this constant is from the DS design
        self.ds_step = rospy.get_param('/stonefish_simulator/ds_distance_step', float)
        # rospy.loginfo(f"DS {self.ds_name} with {self.ds_north},{self.ds_east},{self.ds_down}")
        if self.ds_name == "ds_alpha":
            self.ds_north += self.ds_step
        elif self.ds_name == "ds_beta":
            self.ds_north -= self.ds_step
        elif self.ds_name == "ds_charlie": 
            self.ds_east += self.ds_step
        else: 
            rospy.loginfo(f" The following Docking station {self.ds_name}, is not defined")
            return 0
        rospy.loginfo(f"DS {self.ds_name} position received at this coordinates {self.ds_north},{self.ds_east},{self.ds_down}")
        self.ds_position_recieved = True


    def execute_cb(self, goal):
        """
        Callback executed when the action is called.
        This method starts the docking state machine.
        """
        rospy.loginfo("Docking action started.")
        self.action = goal.action
        print("Docking STATE",)
        # Run the state machine
        self.run_state_machine()

        if self.success:
            # Action succeeded
            self.result.success = True
            self.server.set_succeeded(self.result)
            print(self.result.__getstate__(), self.feedback.__getstate__())
            rospy.loginfo(f"{self.action} action completed successfully. {self.result.success}")
        else:
            # Action failed
            self.result.success = False
            self.server.set_aborted(self.result)
            rospy.loginfo(f"{self.action} action aborted due to failure.")

    def run_state_machine(self):
        """
        Executes the docking state machine sequence.
        Returns True if successful, False otherwise.
        """
        rospy.loginfo(f"Running SM from state id: {self.current_state.id}")

        if self.current_state.id == "initial_pose" and self.action == "dock":
            self.docking_running = True
            self.send("homing_menouver")
        elif self.current_state.id == "in_ds" and self.action == "undock":
            self.send("undock")
        elif self.current_state.id != "initial_pose" and self.action == "dock":
            rospy.loginfo(f"Cannot dock in the current state: {self.current_state.id}")
            self.success = False
        elif self.current_state.id != "in_ds" and self.action == "undock":
            rospy.loginfo(f"Cannot undock in the current state: {self.current_state.id}")
            self.success = False
        else: 
            rospy.loginfo(f"The action {self.action}, does not match the available actions (dock) or (undock)")
            self.success = False

    def feedbackpub(self):
        # Feedback publish
        self.feedback.status = self.current_state.id
        self.server.publish_feedback(self.feedback)
        return 0

    # Block 1
    def on_enter_initial_pose(self):
        while ( not self.ds_position_recieved):
            rospy.loginfo(f"No docking station Information, check the definition of the DS names and parameters")
            self.rate.sleep()
        rospy.loginfo(f"{self.auv_name} docking_action ready, waiting for (dock) action")
   
    def on_exit_initial_pose(self):
        self.feedbackpub()
        self.thrusters_on()
        
    # Block 2
    def on_enter_homing(self):
        self.feedbackpub()
        self.ds_open_locks()
        rospy.loginfo(f"{self.auv_name} started the homming menouver")
        self.set_WWP(z_axis=2)
        self.move_auv_WWP(0.1)
        self.send("docking_menouver")
    
    # Block 3
    def on_enter_entering_ds(self):
        
        rospy.loginfo(f"{self.auv_name} is entering {self.ds_name}")
        self.feedbackpub()
        self.set_WWP(z_axis=-0.1)
        self.move_auv_WWP(0.10)
        self.ds_close_locks()
        self.heave(5,100)
        # code for entering
        self.send("docking_menouver")
        

    def on_exit_entering_ds(self):
        rospy.loginfo(f"{self.auv_name} completed the docking action")
        
    #Block 4
    def on_enter_in_ds(self):
        self.feedbackpub()
        rospy.loginfo(f"Checking if {self.auv_name} is in {self.ds_name}")
        self.thrusters_off()
        rospy.sleep(10)
        self.position_error()
        self.success= False if self.position_mean_error_z>0.15 else True
        if self.success:
            self.dock_attempt = 0 
            rospy.loginfo(f"{self.auv_name} is in {self.ds_name}, use (undock) action to exit the DS")
        elif self.dock_attempt==2:
            self.dock_attempt = 0
            rospy.loginfo(f"{self.auv_name} docking failed {self.ds_name}")
        else:      
            rospy.loginfo(f"{self.auv_name} docking attempt failed {self.ds_name}")
            self.dock_attempt += 1
            self.send("rehome") 
        

    def on_exit_in_ds(self):
        self.thrusters_on()
        pass
        
    #Block 5 
    def on_enter_leave_ds(self):
        self.feedbackpub()
        rospy.loginfo("Undocking Action started")
        self.ds_open_locks()
        self.heave(7,100)
        self.heave(4,-100)
        # undocking 
        self.send("undock")

    def on_exit_leave_ds(self):
        rospy.loginfo(f"{self.auv_name} exited successfully")
        self.success = True

    
    def move_auv_WWP(self,error):
        self.position_error()
        self.world_waypoint_publisher()
        print(f"Target:{np.round(self.ned_target_pose[:-1,-1].T,2)}")
        while (not rospy.is_shutdown() and  self.position_mean_error > error):
            self.position_error()
            self.world_waypoint_publisher() 
            self.yaw_axis_disable = True if self.position_mean_error>3.0 else False
            self.rate.sleep()

    def heave(self,time,force_z):
        time_now = rospy.get_rostime().to_sec()
        while((time_now+time) > rospy.get_rostime().to_sec()):
            value = np.array([0,0,force_z,0])
            self.publish_forces(value)
            self.rate.sleep()

    def position_error(self):
        difference = np.linalg.inv(self.nav_ned_T_b)@self.ned_target_pose
        # error in Z
        self.position_mean_error_z =np.linalg.norm(difference[2,-1])

        self.position_mean_error =np.linalg.norm(difference[:3,-1])

    def thrusters_off(self):
        thrusters_off = rospy.ServiceProxy(f"/{self.auv_name}/controller/disable_thrusters", Trigger)
        rospy.loginfo(f"Thrusters Disabled {thrusters_off()}")
    
    def thrusters_on(self):
        thrusters_on = rospy.ServiceProxy(f"/{self.auv_name}/controller/enable_thrusters", Trigger)
        rospy.loginfo(f"Thrusters enabled {thrusters_on()}")

    def b_T_ds_msg(self,data):
        value = np.array([
            data.pose.pose.position.x,
            data.pose.pose.position.y,
            data.pose.pose.position.z,
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        ],dtype=float)

        self.b_T_ds = self.quat_to_tf(value)

    def AUV_odometry(self, data):
        # if not isinstance(self.nav_ned_T_b, np.ndarray): 
        #     print("Odom Recieved")
        value = np.array([
            data.pose.pose.position.x,
            data.pose.pose.position.y,
            data.pose.pose.position.z,
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        ],dtype=float)

        self.nav_ned_T_b = self.quat_to_tf(value)

    def set_WWP(self,z_axis):
        self.new_coordinates[0] = self.ds_north
        self.new_coordinates[1] = self.ds_east
        self.new_coordinates[2] = self.ds_down - z_axis
        self.new_coordinates[3] = 0.0 

        self.ned_target_pose = self.quat_to_tf(np.array([self.new_coordinates[0],
                                                         self.new_coordinates[1],
                                                         self.new_coordinates[2],
                                                         0,0,0,1]))


    def change_worldwaypoint(self):
        #defining the world request msg
        self.way_req = WorldWaypointReq()
        self.way_req.goal.priority = GoalDescriptor.PRIORITY_NORMAL
        self.way_req.header.stamp = rospy.Time().now()
        self.way_req.goal.requester = "DS_waypoint_Target"
        self.way_req.header.frame_id = "world_ned"
        self.way_req.altitude_mode = False

        self.way_req.position.north = self.new_coordinates[0]
        self.way_req.position.east = self.new_coordinates[1]
        self.way_req.position.depth = self.new_coordinates[2]
        self.way_req.orientation.roll = 0.0
        self.way_req.orientation.pitch = 0.0
        self.way_req.orientation.yaw = self.new_coordinates[3]
        self.way_req.disable_axis.yaw = self.yaw_axis_disable

        self.way_req.position_tolerance.x = self.new_coordinates[4]
        self.way_req.position_tolerance.y = self.new_coordinates[5]
        self.way_req.position_tolerance.z = self.new_coordinates[6]

        self.way_req.orientation_tolerance.roll = 0.03
        self.way_req.orientation_tolerance.pitch = 0.03
        self.way_req.orientation_tolerance.yaw = self.new_coordinates[7]

    def display_waypoint(self): 
        # retriving quatornion from tf 
        ds_pose = PoseWithCovarianceStamped()
        # Header Defining 
        ds_pose.header.frame_id='world_ned'
        ds_pose.header.stamp = rospy.get_rostime()
        # translation position publish
        ds_pose.pose.pose.position.x = self.new_coordinates[0]
        ds_pose.pose.pose.position.y = self.new_coordinates[1]
        ds_pose.pose.pose.position.z = self.new_coordinates[2]
        # publishing quaternion from the ArUco pose estimation 
        ds_pose.pose.pose.orientation.x = 0
        ds_pose.pose.pose.orientation.y = 0
        ds_pose.pose.pose.orientation.z = 0.7071788
        ds_pose.pose.pose.orientation.w = 0.7070348
        
        self.waypoint_display.publish(ds_pose)

    def world_waypoint_publisher(self):
        self.change_worldwaypoint()
        self.req_pub.publish(self.way_req)
        self.display_waypoint()

    def publish_forces(self,value): 
        msg  = BodyForceReq()
        # Header 
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = "girona1000/base_link"
        # goal
        msg.goal.requester="/girona1000/docking_forces"
        msg.goal.priority= 30 
        # position 
        msg.wrench.force.x = value[0]
        msg.wrench.force.y = value[1]
        msg.wrench.force.z = value[2]
        #orientation 
        msg.wrench.torque.z = value[3]
        self.move_force.publish(msg)
        self.display_waypoint()

    def ds_open_locks(self):
        try:
            open_locks = rospy.ServiceProxy(f"/{self.ds_name}/open_locks", Trigger)
            # open_locks = rospy.ServiceProxy("/docking_station/open_locks", Trigger)
            res = open_locks()
            if(res.success):
                rospy.loginfo("... Opening latching system...")
            else:
                rospy.loginfo(f"... Failed to open latching system! {res.message}" )
        except:
            rospy.loginfo("... Service call failed, cannot open Locks ...")

    def ds_close_locks(self):
        try:
            close_locks = rospy.ServiceProxy(f"/{self.ds_name}/close_locks", Trigger)
            # close_locks = rospy.ServiceProxy("/docking_station/close_locks", Trigger)
            res = close_locks()
            if(res.success):
                rospy.loginfo("... Closing latching system...")
                self.auv_locked = True
            else:
                rospy.loginfo(f"... Failed to close latching system! ... {res.message}")
                self.docking_failed = True
                return -1
        except:
            rospy.loginfo("... Service call failed, cannot close Locks....")
            self.docking_failed = True
            return -1

    def quat_to_tf(self,data,only_tf=True):
        eular = tf.transformations.euler_from_quaternion(data[3:])
        # using tf function 
        roll, pitch, yaw = eular[0],eular[1],eular[2]
        transformation_matrix = tf.transformations.euler_matrix(roll, pitch, yaw, 'sxyz') # rotation
        transformation_matrix[:3,-1] =data[:3] # translation
        if only_tf == True:
            return transformation_matrix
        else: 
            return roll, pitch, yaw, transformation_matrix

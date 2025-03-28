from sensor_msgs.msg import JointState
import numpy as np
from std_srvs.srv import Trigger, TriggerResponse
import rospy


class  DSController():
    def __init__(self, ds_name) -> None:
        self.lock_position= [0.0,0.0]
        self.lock_velocity= [0.0,0.0]
        self.ds_name = ds_name
        self.value = None
        self.lock_true =False
        self.unlock_true =False

        self.locks_busy = False

        self.control_lock_pos = rospy.Publisher(self.ds_name+'/my_joint_setpoints',JointState,queue_size=10)
        self.control_lock_vel = rospy.Publisher(self.ds_name+'/my_joint_setpoints',JointState,queue_size=10)

        self.joint_state = rospy.Subscriber(self.ds_name+'/my_joint_state',JointState,self.check_joint_state)

        # define the service
        open_lock_srv = rospy.Service(self.ds_name+'/open_locks', Trigger, self.opening_locks)
        close_lock_srv = rospy.Service(self.ds_name+'/close_locks', Trigger, self.closing_locks)
        
        self.rate = rospy.Rate(25)
        rospy.Timer(rospy.Duration(1), self.lock_unlock)


    def lock_unlock(self,msg):
        if self.lock_true:
            self.lock_true=False
            self.close_locks()
        if self.unlock_true:
            self.unlock_true = False
            self.open_locks()
    
    def opening_locks(self,request):
        if self.locks_busy:
            rospy.loginfo("opening locks in process")
            return TriggerResponse(
            success=True,
            message="opening locks in process"
            ) 

        self.unlock_true = True
        rospy.loginfo("opening locks in process")
        return TriggerResponse(
            success=True,
            message="Hey, opening locks"
        )
    
    def closing_locks(self,request):
        if self.locks_busy:
            rospy.loginfo("closing locks in process")
            return TriggerResponse(
            success=True,
            message="closing locks in process"
            ) 
        self.lock_true = True
        rospy.loginfo("closing locks in process")
        return TriggerResponse(
            success=True,
            message="Hey, closing locks"
        )


    def open_locks(self):
        if self.lock_position[0] < 0.03: 
            rospy.loginfo(f"Locks are opened {self.ds_name}")
            return 0
        else: 
            self.value = 0.15
            target = 0.0 
        self.locks_busy = True
        while self.value>=target: 
            self.value-=0.001
            self.publish_lock_position([self.value,self.value])
            self.rate.sleep()
        self.locks_busy = False

    def close_locks(self):
        if self.lock_position[0] > 0.12: 
            rospy.loginfo(f"Locks are Closed {self.ds_name}")
            return 0
        self.value = 0.0
        target = 0.15 
        self.locks_busy = True
        while self.value<=target: 
            self.value+=0.001
            self.publish_lock_position([self.value,self.value])
            self.rate.sleep()
        self.locks_busy = False

    def set_position(self):
        return self.publish_lock_position()
    
    def set_velocity(self):
        return self.publish_lock_velocity
    
    def check_joint_state(self,msg:JointState):
        # close
        self.lock_position[0] = msg.position[0]
        self.lock_position[1] = msg.position[1]

    def publish_lock_position(self,position):
        msg=JointState()

        msg.header.frame_id = self.ds_name
        msg.header.stamp = rospy.get_rostime()

        msg.name = [self.ds_name+"/lock/lock1",self.ds_name+"/lock/lock2"]
        msg.position=position
        # rospy.loginfo(f"published msg{np.round((msg.position),2)}, for {self.ds_name}")
        self.control_lock_pos.publish(msg)

    def publish_lock_velocity(self,velocity):
        msg=JointState()

        msg.header.frame_id = self.ds_name
        msg.header.stamp = rospy.get_rostime()

        msg.name = [self.ds_name+"/lock/lock1",self.ds_name+"/lock/lock2"]
        msg.velocity =velocity
        self.control_lock_vel.publish(msg)
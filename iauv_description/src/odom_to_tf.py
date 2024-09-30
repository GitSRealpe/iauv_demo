#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry


class Odom_translator:
    param = "nothing"

    def __init__(self):
        self.param = rospy.get_param("~name")
        print("subscribing to" + self.param + "odometry")
        rospy.Subscriber(
            "/" + self.param + "/dynamics/odometry", Odometry, self.callback
        )

    # This node is used to publish tf based on the odometry of the vehicle
    # as the robot publishes the tf through the driver of the INS
    def callback(self, msg):
        br = tf.TransformBroadcaster()
        br.sendTransform(
            (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ),
            (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ),
            rospy.Time.now(),
            "/" + self.param + "/base_link",
            "/world_ned",
        )


def listener():
    rospy.init_node("odom_to_tf", anonymous=True)

    odomt = Odom_translator()

    rospy.spin()


if __name__ == "__main__":
    listener()

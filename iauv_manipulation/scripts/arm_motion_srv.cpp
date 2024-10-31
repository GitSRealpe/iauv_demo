#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/String.h>

#include <iauv_manipulation/Motion.h>

class arm_motion_srv
{
private:
    ros::NodeHandle nh;
    ros::ServiceServer motion_srv;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac;

    std::string srv_name;
    std::string node_name;

public:
    arm_motion_srv(/* args */);
    ~arm_motion_srv();

    bool motion_request(iauv_manipulation::Motion::Request &req, iauv_manipulation::Motion::Response &res);
};

arm_motion_srv::arm_motion_srv(/* args */) : ac("/girona1000/bravo/joint_trajectory_controller/follow_joint_trajectory", true)
{
    ROS_INFO("Initializing arm motions service");

    node_name = ros::this_node::getName();
    nh.getParam(node_name + "/srv_name", srv_name);

    motion_srv = nh.advertiseService("/arm_motion_srv", &arm_motion_srv::motion_request, this);

    ROS_INFO("Waiting for arm controler action server to start.");
    // wait for the action server to start
    ac.waitForServer(); // will wait for infinite time
    ROS_INFO("Arm controler action server active.");
}

arm_motion_srv::~arm_motion_srv()
{
    ROS_INFO("Finishin arm motions service");
    ac.cancelAllGoals();
    // ac.~SimpleActionClient();
}

bool arm_motion_srv::motion_request(iauv_manipulation::Motion::Request &req, iauv_manipulation::Motion::Response &res)
{
    ROS_INFO("Requested [%s] arm motion", req.request.c_str());

    control_msgs::FollowJointTrajectoryGoal joint_cmd;

    nh.getParam("motions/" + req.request + "/joint_names", joint_cmd.trajectory.joint_names);

    XmlRpc::XmlRpcValue points;
    nh.getParam("motions/" + req.request + "/points", points);
    std::cout << points.size() << "\n";
    joint_cmd.trajectory.points.resize(points.size());
    for (int i = 0; i < points.size(); i++)
    {
        XmlRpc::XmlRpcValue positions = points[i]["positions"];
        XmlRpc::XmlRpcValue velocities = points[i]["velocities"];

        for (size_t j = 0; j < positions.size(); j++)
        {
            joint_cmd.trajectory.points[i].positions.push_back(positions[j]);
            joint_cmd.trajectory.points[i].velocities.push_back(velocities[j]);
        }

        // joint_cmd.trajectory.points.at(i).time_from_start.sec

        joint_cmd.trajectory.points[i].time_from_start.sec = static_cast<double>(points[i]["time_from_start"]["secs"]);
        joint_cmd.trajectory.points[i].time_from_start.nsec = static_cast<double>(points[i]["time_from_start"]["nsecs"]);
    }

    std::cout << joint_cmd << "\n";

    ac.sendGoal(joint_cmd);

    bool finished = ac.waitForResult(ros::Duration(10.0));

    res.response = finished ? "motion done" : "motion not done";

    return finished;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_motion_srv");

    arm_motion_srv armsrv;

    while (ros::ok())
    {
        // ros::spinOnce();
        // ros::Duration(1).sleep();
        ros::spin();
    }
}
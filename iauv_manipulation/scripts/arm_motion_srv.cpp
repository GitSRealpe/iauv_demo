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

    std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> ac;

    std::string srv_name;
    std::string ctrl;
    std::string node_name;

public:
    arm_motion_srv(/* args */);
    ~arm_motion_srv();

    bool motion_request(iauv_intervention_msgs::Motion::Request &req, iauv_intervention_msgs::Motion::Response &res);
};

arm_motion_srv::arm_motion_srv(/* args */)
{
    ROS_INFO("Initializing arm motions service");

    node_name = ros::this_node::getName();
    nh.getParam(node_name + "/srv_name", srv_name);
    motion_srv = nh.advertiseService(srv_name, &arm_motion_srv::motion_request, this);

    nh.getParam(node_name + "/controller", ctrl);
    ac = std::make_shared<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>(ctrl + "/follow_joint_trajectory", true);
    ROS_INFO("Waiting for arm controler action server to start.");
    // wait for the action server to start
    ac->waitForServer();
    ROS_INFO("Arm controler action server active.");

    XmlRpc::XmlRpcValue motions;
    nh.getParam("motions/", motions);
    for (int i = 0; i < motions.size(); i++)
    {
        std::cout << motions[i]["motion_name"] << "\n";
    }
}

arm_motion_srv::~arm_motion_srv()
{
    ROS_INFO("Finishing arm motions service");
    ac->cancelAllGoals();
    // ac.~SimpleActionClient();
}

bool arm_motion_srv::motion_request(iauv_intervention_msgs::Motion::Request &req, iauv_intervention_msgs::Motion::Response &res)
{
    ROS_INFO("Requested [%s] arm motion", req.request.c_str());

    XmlRpc::XmlRpcValue motions;
    nh.getParam("motions/", motions);
    int m = 0;
    for (; m < motions.size(); m++)
    {
        if (motions[m]["motion_name"] == req.request)
        {
            std::cout << "found: " << motions[m]["motion_name"] << "\n";
            break;
        }
    }

    if (m == motions.size())
    {
        std::cout << "motion not found bro" << "\n";
        res.response = "motion not found in config file";
        return false;
    }

    control_msgs::FollowJointTrajectoryGoal joint_cmd;
    for (int i = 0; i < motions[m]["joint_names"].size(); i++)
    {
        joint_cmd.trajectory.joint_names.push_back(motions[m]["joint_names"][i]);
    }

    XmlRpc::XmlRpcValue points = motions[m]["points"];
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

    ac->sendGoal(joint_cmd);

    bool finished = ac->waitForResult(ros::Duration(10.0));

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
#include <ros/ros.h>
#include <stdio.h>

#include <iauv_motion_planner/GetPath.h>
#include <iauv_motion_planner/Scene.h>

// planners
#include <iauv_motion_planner/ScanPlanner.h>
#include <iauv_motion_planner/sbmp/RRTPlanner.h>
#include <iauv_motion_planner/CircularPlanner.h>

// #include <tf2_ros/transform_listener.h>

class iauv_motion_planner_node
{
private:
    iauv_motion_planner::ScanPlannerPtr scan;
    iauv_motion_planner::RRTPlannerPtr rrt;
    iauv_motion_planner::CircPlannerPtr circ;
    iauv_motion_planner::ScenePtr scn;

    // geometry_msgs::TransformStamped t;
    // tf2_ros::TransformListener tfListener;
    // tf2_ros::Buffer tfBuffer;

    ros::Publisher pub;
    ros::ServiceServer service_;

public:
    iauv_motion_planner_node(ros::NodeHandle &nh);

    nav_msgs::PathPtr path;

    bool getPath(iauv_motion_planner::GetPath::Request &req, iauv_motion_planner::GetPath::Response &res);
};

iauv_motion_planner_node::iauv_motion_planner_node(ros::NodeHandle &nh) //: tfListener(tfBuffer)
{

    scn = std::make_shared<iauv_motion_planner::Scene>();
    scan = std::make_shared<iauv_motion_planner::ScanPlanner>(nh);
    // rrt = std::make_shared<iauv_motion_planner::RRTPlanner>(nh);
    circ = std::make_shared<iauv_motion_planner::CircPlanner>(nh);

    pub = nh.advertise<nav_msgs::Path>("/iauv_motion_planner/path", 2, true);
    service_ = nh.advertiseService("getPath", &iauv_motion_planner_node::getPath, this);
}

bool iauv_motion_planner_node::getPath(iauv_motion_planner::GetPath::Request &req, iauv_motion_planner::GetPath::Response &res)
{
    std::cout << "service called\n";
    std::cout << "received request" << "\n";
    std::cout << req << "\n";

    Eigen::Quaternionf q(req.start.orientation.w,
                         req.start.orientation.x,
                         req.start.orientation.y,
                         req.start.orientation.z);

    std::vector<double> start = {req.start.position.x,
                                 req.start.position.y,
                                 req.start.position.z,
                                 atan2(q.toRotationMatrix()(1, 0), q.toRotationMatrix()(0, 0))};

    q = Eigen::Quaternionf(req.goal.orientation.w,
                           req.goal.orientation.x,
                           req.goal.orientation.y,
                           req.goal.orientation.z);

    std::vector<double> goal = {req.goal.position.x,
                                req.goal.position.y,
                                req.goal.position.z,
                                atan2(q.toRotationMatrix()(1, 0), q.toRotationMatrix()(0, 0))};

    // no usar este angle y usar el goal[3]
    double angle = atan2(q.toRotationMatrix()(1, 0), q.toRotationMatrix()(0, 0));
    std::vector<double> prepath;

    switch (req.planner)
    {
    case iauv_motion_planner::GetPathRequest::RRT:
        std::cout << start[0] << "before do plan \n";
        res.path = rrt->doPlan(start, goal);
        break;
    case iauv_motion_planner::GetPathRequest::SCANNER:
        if (!scan->checkParams(req.params))
        {
            break;
        }

        prepath = {req.goal.position.x - (scan->length_ / 2),
                   req.goal.position.y - (scan->width_ / 2),
                   req.goal.position.z,
                   angle + 3.1415};

        // scan->path_ = rrt->doPlan(start, prepath);
        res.path = scan->doPlan(goal);
        break;
    case iauv_motion_planner::GetPathRequest::CIRCULAR:
        if (!circ->checkParams(req.params))
        {
            break;
        }

        prepath = {req.goal.position.x + ((circ->radius_) * cos(angle)),
                   req.goal.position.y + ((circ->radius_) * sin(angle)),
                   req.goal.position.z,
                   angle + 3.1415};
        // circ->path_ = rrt->doPlan(start, prepath);
        res.path = circ->doPlan(goal, goal);
        break;
    default:
        std::cout << "planner switch defaulted\n";
        break;
    }

    pub.publish(res.path);
    // scn->drawPath(res.path);

    return true;
}

int main(int argc, char **argv)
{
    std::cout << "yeah planning\n";
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh("~");

    iauv_motion_planner_node node(nh);

    while (ros::ok())
    {
        // ros::spinOnce();
        // ros::Duration(1).sleep();
        ros::spin();
    }

    return 0;
}

#include <ros/ros.h>
#include <stdio.h>

#include <iauv_motion_planner/GetPath.h>
#include <iauv_motion_planner/Scene.h>

// planners
#include <iauv_motion_planner/SimplePlanner.h>
#include <iauv_motion_planner/ScanPlanner.h>
#include <iauv_motion_planner/CircularPlanner.h>

class iauv_motion_planner_node
{
private:
    iauv_motion_planner::ScanPlannerPtr scan;
    iauv_motion_planner::SimplePlannerPtr simple;
    iauv_motion_planner::CircPlannerPtr circ;
    // iauv_motion_planner::ScenePtr scn;

    ros::Publisher pub;
    ros::ServiceServer service_;

public:
    iauv_motion_planner_node(ros::NodeHandle &nh);

    nav_msgs::PathPtr path;

    bool getPath(iauv_motion_planner::GetPath::Request &req, iauv_motion_planner::GetPath::Response &res);
};

iauv_motion_planner_node::iauv_motion_planner_node(ros::NodeHandle &nh)
{

    // scn = std::make_shared<iauv_motion_planner::Scene>();
    simple = std::make_shared<iauv_motion_planner::SimplePlanner>(nh);
    scan = std::make_shared<iauv_motion_planner::ScanPlanner>(nh);
    circ = std::make_shared<iauv_motion_planner::CircPlanner>(nh);

    pub = nh.advertise<nav_msgs::Path>("/iauv_motion_planner/path", 2, true);
    service_ = nh.advertiseService("getPath", &iauv_motion_planner_node::getPath, this);
}

bool iauv_motion_planner_node::getPath(iauv_motion_planner::GetPath::Request &req, iauv_motion_planner::GetPath::Response &res)
{
    std::cout << "service called\n";
    std::cout << "received request" << "\n";
    // std::cout << req << "\n";

    std::cout << "In frame: " << req.header.frame_id << "\n";
    std::cout << "  Start: "
              << "X: " << req.start.position.x
              << ", Y: " << req.start.position.y
              << ", Z: " << req.start.position.z << "\n";
    std::cout << "  Goal: "
              << "X: " << req.goal.position.x
              << ", Y: " << req.goal.position.y
              << ", Z: " << req.goal.position.z << "\n";

    Eigen::Quaternionf q(req.start.orientation.w, req.start.orientation.x,
                         req.start.orientation.y, req.start.orientation.z);

    std::vector<double> start = {req.start.position.x, req.start.position.y, req.start.position.z,
                                 atan2(q.toRotationMatrix()(1, 0), q.toRotationMatrix()(0, 0))};

    q = Eigen::Quaternionf(req.goal.orientation.w, req.goal.orientation.x,
                           req.goal.orientation.y, req.goal.orientation.z);

    std::vector<double> goal = {req.goal.position.x, req.goal.position.y, req.goal.position.z,
                                atan2(q.toRotationMatrix()(1, 0), q.toRotationMatrix()(0, 0))};

    // no usar este angle y usar el goal[3]
    double angle = atan2(q.toRotationMatrix()(1, 0), q.toRotationMatrix()(0, 0));
    std::vector<double> prepath;

    // nav_msgs::PathPtr path;
    // path->header = req.header;

    switch (req.planner)
    {
    case iauv_motion_planner::GetPathRequest::SIMPLE:
        if (!simple->checkParams(req.params))
        {
            break;
        }
        res.path = simple->doPlan(start, goal);
        res.path.header = req.header;
        break;
    case iauv_motion_planner::GetPathRequest::SCANNER:
        if (!scan->checkParams(req.params))
        {
            break;
        }
        prepath = {req.goal.position.x - (scan->params_["length"] / 2),
                   req.goal.position.y - (scan->params_["width"] / 2),
                   req.goal.position.z,
                   angle + 3.1415};

        // simple->path_ = simple->doPlan(start, prepath);
        scan->path_ = simple->doPlan(start, prepath);
        scan->path_.header = req.header;
        res.path = scan->doPlan(goal);
        break;
    case iauv_motion_planner::GetPathRequest::CIRCULAR:
        if (!circ->checkParams(req.params))
        {
            break;
        }
        prepath = {req.goal.position.x + ((circ->params_["radius"]) * cos(angle)),
                   req.goal.position.y + ((circ->params_["radius"]) * sin(angle)),
                   req.goal.position.z,
                   angle + 3.1415};
        std::cout << "\n";

        circ->path_ = simple->doPlan(start, prepath);
        circ->path_.header = req.header;
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

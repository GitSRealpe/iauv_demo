#ifndef PLANNER_
#define PLANNER_

#include <string>
#include <ros/ros.h>
#include <nav_msgs/Path.h>

// #include <iauv_motion_planner/GetPath.h>
// #include <iauv_motion_planner/Scene.h>
#include <iauv_motion_planner/classForwards.h>
#include <iauv_motion_planner/PlannerParam.h>

// fcl stuff
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_request.h>
#include <fcl/narrowphase/collision_result.h>

using CollObjPtr = std::shared_ptr<fcl::CollisionObjectf>;
using CollGeomPtr = std::shared_ptr<fcl::CollisionGeometryf>;

namespace iauv_motion_planner
{
    // IAUVPLANNER_CLASS_FORWARD(Planner);

    class Planner
    {
    private:
        std::string map_topic_;
        ros::NodeHandle nh_;

    public:
        Planner(ros::NodeHandle &nh);

        virtual nav_msgs::Path doPlan(std::vector<double> start, std::vector<double> goal)
        {
        }

        virtual nav_msgs::Path doPlan(std::vector<double> center)
        {
        }

        bool checkParams(std::vector<iauv_motion_planner::PlannerParam> params);

        nav_msgs::Path path_;
        std::string planner_name;
        std::map<std::string, double> params_;
    };

}

#endif
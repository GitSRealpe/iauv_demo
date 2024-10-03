#include <iauv_motion_planner/Planner.h>
#include <iauv_motion_planner/GetPath.h>
#include <iostream>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

// fcl stuff
#include <fcl/geometry/octree/octree.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_request.h>
#include <fcl/narrowphase/collision_result.h>

#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>

namespace iauv_motion_planner
{

    Planner::Planner(ros::NodeHandle &nh) : nh_(nh)
    {
        // std::cout << "planner default\n";
        // std::cout << nh.getNamespace() << "\n\n";
        // params_["segment_length"] = 0.3;
    }

    bool Planner::checkParams(std::vector<iauv_motion_planner::PlannerParam> req_params)
    {
        std::cout << "checking [" << planner_name << "] params:" << "\n";
        for (auto &param : req_params)
        {
            auto it = params_.find(param.key);
            if (it != params_.end())
            {
                // If key is found, modify the value
                it->second = std::stod(param.value);
                std::cout << "  " << param.key << ": " << it->second << "\n";
            }
            else
            {
                std::cout << "  " << param.key << " not defined as [" << planner_name << "] parameter \n";
                std::cout << "Available params for: " << "[" << planner_name << "] are: \n";
                for (auto &item : params_)
                {
                    std::cout << item.first << ", ";
                }
                std::cout << "\n";

                return false;
            }
        }
        return true;
    }

}
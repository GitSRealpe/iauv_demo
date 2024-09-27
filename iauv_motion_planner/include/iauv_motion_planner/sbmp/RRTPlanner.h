#ifndef RRTPLANNER_
#define RRTPLANNER_

#include <string>
#include <iauv_motion_planner/sbmp/SampleBasedPlanner.h>
// #include <iauv_motion_planner/sbmp/ValidatorConstrained.h>
#include <iauv_motion_planner/sbmp/Validator3.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace iauv_motion_planner
{
    IAUVPLANNER_CLASS_FORWARD(RRTPlanner);
    class RRTPlanner : public SampleBasedPlanner
    {
    private:
    public:
        RRTPlanner(ros::NodeHandle &nh);
        nav_msgs::Path doPlan(std::vector<double> start, std::vector<double> goal);
        std::shared_ptr<ob::RealVectorStateSpace> navSpace_;

        Validator3Ptr val3_ptr;
    };

}

#endif
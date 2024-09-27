#ifndef CIRCPLANNER_
#define CIRCPLANNER_

#include <iauv_motion_planner/Planner.h>

namespace iauv_motion_planner
{
    IAUVPLANNER_CLASS_FORWARD(CircPlanner);
    class CircPlanner : public Planner
    {
    private:
    public:
        CircPlanner(ros::NodeHandle &nh);
        // ~SampleBasedPlanner();

        nav_msgs::Path doPlan(std::vector<double> start, std::vector<double> goal);
        void forward(Eigen::Isometry3d mov, int steps);

        Eigen::Isometry3d next_;

        double radius;
    };

} // namespace iauv_motion_planner

#endif
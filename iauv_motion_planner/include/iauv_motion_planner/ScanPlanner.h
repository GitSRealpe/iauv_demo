#ifndef SCANPLANNER_
#define SCANPLANNER_

#include <iauv_motion_planner/Planner.h>

namespace iauv_motion_planner
{
    IAUVPLANNER_CLASS_FORWARD(ScanPlanner);
    class ScanPlanner : public Planner
    {
    private:
    public:
        ScanPlanner(ros::NodeHandle &nh);
        // ~SampleBasedPlanner();

        nav_msgs::Path doPlan(std::vector<double> center);
        void forward(Eigen::Isometry3d mov, int steps);

        Eigen::Isometry3d next_;

        double width;
        double length;
    };

} // namespace iauv_motion_planner

#endif
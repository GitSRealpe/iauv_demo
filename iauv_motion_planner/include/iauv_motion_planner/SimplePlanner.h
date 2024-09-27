#ifndef SIMPLANNER
#define SIMPLANNER

#include <iauv_motion_planner/Planner.h>

namespace iauv_motion_planner
{
    IAUVPLANNER_CLASS_FORWARD(SimplePlanner);

    class SimplePlanner : public Planner
    {
    private:
    public:
        SimplePlanner(ros::NodeHandle &nh);
        nav_msgs::Path doPlan(std::vector<double> start, std::vector<double> goal);
    };

} // namespace iauv_motion_planner

#endif
#ifndef SBMPLANNER_
#define SBMPLANNER_

#include <iauv_motion_planner/Planner.h>
#include <iauv_motion_planner/sbmp/Validator.h>

// ompl stuff
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

// fcl stuff
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_request.h>
#include <fcl/narrowphase/collision_result.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace iauv_motion_planner
{

    class SampleBasedPlanner : public Planner
    {
    private:
    public:
        SampleBasedPlanner(ros::NodeHandle &nh);
        // ~SampleBasedPlanner();

        CollObjPtr updateMap();
        // fcl::CollisionObjectf updateMap();
        // nav_msgs::Path path_;

        ob::StateSpacePtr navSpace_;
        ompl::base::ProblemDefinitionPtr pdef_;
        ob::PlannerPtr planner_;
        og::PathSimplifierPtr simply;
        og::SimpleSetupPtr ss_;

        CollObjPtr mapColl;

        ValidatorPtr val_ptr;
    };

} // namespace iauv_motion_planner

#endif
#ifndef VALIDATOR3_
#define VALIDATOR3_

#include <iauv_motion_planner/classForwards.h>
#include <ompl/base/StateValidityChecker.h>

// fcl stuff
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_request.h>
#include <fcl/narrowphase/collision_result.h>

namespace ob = ompl::base;

using CollObjPtr = std::shared_ptr<fcl::CollisionObjectf>;
using CollGeomPtr = std::shared_ptr<fcl::CollisionGeometryf>;

namespace iauv_motion_planner
{
    IAUVPLANNER_CLASS_FORWARD(Validator3);
    class Validator3 : public ob::StateValidityChecker
    {
    private:
        // mutable bool printed = false;

    public:
        Validator3(const ob::SpaceInformationPtr &si, CollObjPtr map);
        virtual bool isValid(const ob::State *state) const;

        // fcl
        // std::shared_ptr<fcl::Boxf> auv_box_;
        std::shared_ptr<fcl::Cylinderf> auv_box_;
        CollObjPtr auv_co_;
        // CollObjPtr tree_obj_;
        CollObjPtr map_co;
    };
}

#endif // VALIDATOR3_
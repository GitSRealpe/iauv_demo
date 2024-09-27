#include <iauv_motion_planner/sbmp/Validator3.h>
#include <ompl/base/ScopedState.h>

namespace ob = ompl::base;

namespace iauv_motion_planner
{

    // creo que el simple setup usa el validator3 una vez, como el tree esta void, tira error
    Validator3::Validator3(const ob::SpaceInformationPtr &si, CollObjPtr mapColl) : ob::StateValidityChecker(si)
    {
        std::cout << "normal validator3 initializing\n";
        // radius, lenght
        auv_box_ = std::make_shared<fcl::Cylinderf>(1.0, 1.6);
        auv_co_ = std::make_shared<fcl::CollisionObjectf>(auv_box_);

        std::cout << "mapcoll type " << mapColl->getNodeType() << "\n";
        std::cout << mapColl << "\n";
        map_co = mapColl;

        std::cout << map_co << "\n";

        std::cout << "done validator3 initializing\n";
    }
    bool Validator3::isValid(const ob::State *state) const
    {

        // std::cout << "validandoo beep boop\n";
        ob::ScopedState<> stt(si_->getStateSpace(), state);
        // stt.print();
        auv_co_->setTranslation(Eigen::Vector3f(stt.reals().at(0), stt.reals().at(1), stt.reals().at(2)));
        auv_co_->setRotation(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()).matrix());
        // std::cout << auv_co_->getTranslation() << "\n";
        fcl::CollisionRequestf col_req_;
        fcl::CollisionResultf col_res_;
        fcl::collide(map_co.get(), auv_co_.get(), col_req_, col_res_);

        return col_res_.isCollision() ? false : true;
        // return true ? false : true;
    }

} // namespace iauv_motion_planner
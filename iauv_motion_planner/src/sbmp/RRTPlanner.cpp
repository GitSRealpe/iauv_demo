#include <iauv_motion_planner/sbmp/RRTPlanner.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/PlannerData.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/base/ScopedState.h>

namespace ob = ompl::base;

namespace iauv_motion_planner
{
    RRTPlanner::RRTPlanner(ros::NodeHandle &nh) : SampleBasedPlanner(nh)
    {
        // create a state space
        navSpace_ = std::make_shared<ob::RealVectorStateSpace>(3);
        navSpace_->setName("Position");
        navSpace_->setDimensionName(0, "X");
        navSpace_->setDimensionName(1, "Y");
        navSpace_->setDimensionName(2, "Z");
        ob::RealVectorBounds bounds(3);
        bounds.setLow(-50);
        bounds.setHigh(50);
        bounds.setLow(2, 0.5);
        bounds.setHigh(2, 15);
        navSpace_->setBounds(bounds);
        navSpace_->setName("navSpace");
        navSpace_->setLongestValidSegmentFraction(0.1);

        std::cout << "setting simple setup\n";
        ss_ = std::make_shared<og::SimpleSetup>(navSpace_);
        auto si = ss_->getSpaceInformation();
        std::cout << "setting validity checker setup\n";
        val3_ptr = Validator3Ptr(new Validator3(si, this->updateMap()));
        ss_->setStateValidityChecker(val3_ptr);

        std::cout << "setting planner \n";
        planner_ = (std::make_shared<og::RRT>(si));
        // planner_->params().setParam("range", "1");

        ss_->setPlanner(planner_);

        std::cout << "printing setup\n";
        ss_->setup();
        ss_->print();
        std::cout << "done printing setup\n";
        std::cout << "fin del constructor\n";
        std::cout << nh.getNamespace() << "\n\n";
    }

    nav_msgs::Path RRTPlanner::doPlan(std::vector<double> start, std::vector<double> goal)
    {
        std::cout << "actually planning\n";
        // this->updateMap();
        val3_ptr->map_co = this->updateMap();
        this->ss_->clear();

        ob::ScopedState<> start_stt(navSpace_);
        start_stt[0] = start[0];
        start_stt[1] = start[1];
        start_stt[2] = start[2];

        ob::ScopedState<> goal_stt(navSpace_);
        goal_stt[0] = goal[0];
        goal_stt[1] = goal[1];
        goal_stt[2] = goal[2];
        std::cout << "requesting solution to:\n\n";
        ss_->setStartAndGoalStates(start_stt, goal_stt);
        ss_->print();

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        // attempt to solve the problem
        ob::PlannerStatus status = ss_->solve(5.0);
        std::cout << status << "\n";
        if (status)
        {
            std::cout << "Found solution:\n";
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

            og::PathGeometric pathres = ss_->getSolutionPath();
            std::cout << pathres.getStateCount() << "\n";
            ss_->simplifySolution(5);
            pathres = ss_->getSolutionPath();
            std::cout << pathres.getStateCount() << "\n";
            pathres.interpolate(pathres.getStateCount() * 10);

            geometry_msgs::PoseStamped posestmp;
            path_ = nav_msgs::Path();
            path_.header.frame_id = "world_ned";
            ob::ScopedState<> last_state(navSpace_);
            last_state = start_stt;
            for (auto stt : pathres.getStates())
            {
                ob::ScopedState<ob::CompoundStateSpace> sstt(navSpace_, stt);
                posestmp.pose.position.x = sstt.reals().at(0);
                posestmp.pose.position.y = sstt.reals().at(1);
                posestmp.pose.position.z = sstt.reals().at(2);

                Eigen::Quaterniond q(Eigen::AngleAxisd(atan2(sstt.reals().at(1) - last_state.reals().at(1),
                                                             sstt.reals().at(0) - last_state.reals().at(0)),
                                                       Eigen::Vector3d::UnitZ()));
                // Eigen::Quaterniond q(Eigen::AngleAxisd(sstt.reals().at(3), Eigen::Vector3d::UnitZ()));
                posestmp.pose.orientation.x = q.x();
                posestmp.pose.orientation.y = q.y();
                posestmp.pose.orientation.z = q.z();
                posestmp.pose.orientation.w = q.w();
                path_.poses.push_back(posestmp);

                last_state = sstt;
            }

            // start
            posestmp.pose.position.x = start[0];
            posestmp.pose.position.y = start[1];
            posestmp.pose.position.z = start[2];
            Eigen::Quaterniond qs(Eigen::AngleAxisd(start[3], Eigen::Vector3d::UnitZ()));

            posestmp.pose.orientation.x = qs.x();
            posestmp.pose.orientation.y = qs.y();
            posestmp.pose.orientation.z = qs.z();
            posestmp.pose.orientation.w = qs.w();
            path_.poses.at(0) = posestmp;
            // goal
            posestmp.pose.position.x = goal[0];
            posestmp.pose.position.y = goal[1];
            posestmp.pose.position.z = goal[2];
            Eigen::Quaterniond qg(Eigen::AngleAxisd(goal[3], Eigen::Vector3d::UnitZ()));
            posestmp.pose.orientation.x = qg.x();
            posestmp.pose.orientation.y = qg.y();
            posestmp.pose.orientation.z = qg.z();
            posestmp.pose.orientation.w = qg.w();
            path_.poses.at(path_.poses.size() - 1) = posestmp;
        }
        else
        {
            std::cout << "Solution not found\n";
        }
        std::cout << "done planning\n";

        return path_;
    }
}
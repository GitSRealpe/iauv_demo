#include <iauv_motion_planner/SimplePlanner.h>

namespace iauv_motion_planner
{
    SimplePlanner::SimplePlanner(ros::NodeHandle &nh) : Planner(nh)
    {

        planner_name = "SimplePlanner";
        std::cout << "loaded simple planner\n";
    }

    nav_msgs::Path SimplePlanner::doPlan(std::vector<double> start, std::vector<double> goal)
    {
        std::cout << "computing path\n";
        path_ = nav_msgs::Path();
        path_.header.frame_id = "map";

        Eigen::Isometry3d baseT(Eigen::Translation3d(start[0], start[1], start[2]) *
                                Eigen::AngleAxisd(start[3], Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond baseq(baseT.rotation());
        Eigen::Isometry3d goalT(Eigen::Translation3d(goal[0], goal[1], goal[2]) *
                                Eigen::AngleAxisd(goal[3], Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond goalq(goalT.rotation());

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = start[0];
        pose.pose.position.y = start[1];
        pose.pose.position.z = start[2];
        pose.pose.orientation.x = baseq.x();
        pose.pose.orientation.y = baseq.y();
        pose.pose.orientation.z = baseq.z();
        pose.pose.orientation.w = baseq.w();
        path_.poses.push_back(pose);

        double dist = 0.3;

        auto diff = baseT.inverse() * goalT;
        // direction vector from start to goal
        Eigen::Vector3d dir = diff.translation();
        // std::cout << dir.norm() << "\n";
        int steps = dir.norm() / dist;
        dir.stableNormalize();

        // forward
        Eigen::Isometry3d fwd(Eigen::Translation3d(dir * 0.3));
        // fwd = Eigen::Translation3d(dir * 0.3);
        Eigen::Isometry3d next = baseT;

        Eigen::Quaterniond q(Eigen::AngleAxisd(atan2(dir[1], dir[0]), Eigen::Vector3d::UnitZ()));

        // Eigen::Quaterniond q(dir);
        for (int i = 0; i < steps; i++)
        {
            next = next * fwd;

            pose = geometry_msgs::PoseStamped();
            pose.pose.position.x = next.translation().x();
            pose.pose.position.y = next.translation().y();
            pose.pose.position.z = next.translation().z();

            // auto q = Eigen::Quaterniond(fwd.rotation());
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();
            path_.poses.push_back(pose);
        }

        pose = geometry_msgs::PoseStamped();
        pose.pose.position.x = goal[0];
        pose.pose.position.y = goal[1];
        pose.pose.position.z = goal[2];
        pose.pose.orientation.x = goalq.x();
        pose.pose.orientation.y = goalq.y();
        pose.pose.orientation.z = goalq.z();
        pose.pose.orientation.w = goalq.w();

        path_.poses.push_back(pose);

        std::cout << "done planning\n";

        return path_;
    }
}
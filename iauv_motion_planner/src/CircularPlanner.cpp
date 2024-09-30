#include <iauv_motion_planner/CircularPlanner.h>

namespace iauv_motion_planner
{
    CircPlanner::CircPlanner(ros::NodeHandle &nh) : Planner(nh)
    {
        planner_name = "CircularPlanner";
        params_["radius"] = 1;
        std::cout << "loaded circular planner\n";
    }

    void CircPlanner::forward(Eigen::Isometry3d mov, int steps)
    {
        geometry_msgs::PoseStamped pose;
        for (int i = 0; i < steps; i++)
        {
            next_ = next_ * mov;
            auto q = Eigen::Quaterniond(next_.rotation());
            pose = geometry_msgs::PoseStamped();
            pose.pose.position.x = next_.translation().x();
            pose.pose.position.y = next_.translation().y();
            pose.pose.position.z = next_.translation().z();
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();
            path_.poses.push_back(pose);
        }
    }

    nav_msgs::Path CircPlanner::doPlan(std::vector<double> start, std::vector<double> goal)
    {
        radius_ = params_["radius"];
        path_ = nav_msgs::Path();
        path_.header.frame_id = "map";

        Eigen::Affine3d baseT(Eigen::Translation3d(start[0], start[1], start[2]) *
                              Eigen::AngleAxisd(start[3], Eigen::Vector3d::UnitZ()));
        // Eigen::Quaterniond baseq(baseT.rotation());

        // ---------------------------------------------------------------
        geometry_msgs::PoseStamped poseS;

        Eigen::Affine3d poseT;
        double arcStep = 10 * M_PI / 180;
        // double range = 0;
        for (double theta = 0; theta < 2 * M_PI; theta += arcStep)
        {
            // these two can be one line, checl l8r
            Eigen::Affine3d poseC(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
            poseC.translation() = Eigen::Vector3d(cos(theta) * radius_, sin(theta) * radius_, 0);

            poseT = baseT * poseC;

            poseS.pose.position.x = poseT.translation().x();
            poseS.pose.position.y = poseT.translation().y();
            poseS.pose.position.z = poseT.translation().z();

            poseT.rotate(Eigen::AngleAxisd(3.1415, Eigen::Vector3d::UnitZ()));
            Eigen::Quaterniond q(poseT.rotation());
            poseS.pose.orientation.x = q.x();
            poseS.pose.orientation.y = q.y();
            poseS.pose.orientation.z = q.z();
            poseS.pose.orientation.w = q.w();
            path_.poses.push_back(poseS);
        }

        return path_;
    }
}
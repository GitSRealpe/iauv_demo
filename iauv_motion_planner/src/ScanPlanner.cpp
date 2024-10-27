#include <iauv_motion_planner/ScanPlanner.h>

namespace iauv_motion_planner
{
    ScanPlanner::ScanPlanner(ros::NodeHandle &nh) : Planner(nh)
    {
        planner_name = "ScanPlanner";
        params_["width"] = 5;
        params_["length"] = 5;
        std::cout << "loaded scan planner\n";
    }

    void ScanPlanner::forward(Eigen::Isometry3d mov, int steps)
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

    nav_msgs::Path ScanPlanner::doPlan(std::vector<double> center)
    {

        width_ = params_["width"];
        length_ = params_["length"];
        // std::cout << "scanarea of width and length: " << width << ", " << length << "\n";

        // path_ = nav_msgs::Path();
        // path_.header.frame_id = "map";

        Eigen::Isometry3d centerT(Eigen::Translation3d(center[0], center[1], center[2]) *
                                  Eigen::AngleAxisd(center[3], Eigen::Vector3d::UnitZ()));

        Eigen::Isometry3d startT(Eigen::Translation3d(-length_ / 2, -width_ / 2, 0));
        Eigen::Isometry3d goalT(Eigen::Translation3d(length_ / 2, width_ / 2, 0));

        Eigen::Isometry3d start_pose = centerT * startT;
        Eigen::Quaterniond startq(start_pose.rotation());

        Eigen::Isometry3d end_pose = centerT * goalT;
        Eigen::Quaterniond endq(end_pose.rotation());

        auto diff = start_pose.inverse() * end_pose;

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = start_pose.translation()[0];
        pose.pose.position.y = start_pose.translation()[1];
        pose.pose.position.z = start_pose.translation()[2];
        pose.pose.orientation.x = startq.x();
        pose.pose.orientation.y = startq.y();
        pose.pose.orientation.z = startq.z();
        pose.pose.orientation.w = startq.w();
        path_.poses.push_back(pose);

        double dist = 0.5;
        int steps_x = length_ / dist;
        int steps_y = 1.5 / dist;
        // std::cout << diff.translation() << "\n";
        // std::cout << diff.translation() / dist << "\n";
        // std::cout << steps_x << "\n";
        // std::cout << steps_y << "\n";

        // forward
        Eigen::Isometry3d fwd;
        fwd = Eigen::Translation3d(dist, 0, 0);
        next_ = start_pose;
        forward(fwd, steps_x);

        double dir = 1;
        double times = diff.translation()[1] / (dist * steps_y) - 1;
        for (size_t i = 0; i <= times; i++)
        {
            // turn
            fwd = Eigen::Translation3d(0, 0, 0) * Eigen::AngleAxisd(1.57 * dir, Eigen::Vector3d::UnitZ());
            next_ = next_ * fwd;
            // forward
            fwd = Eigen::Translation3d(dist, 0, 0);
            forward(fwd, steps_y);
            // turn
            fwd = Eigen::Translation3d(0, 0, 0) * Eigen::AngleAxisd(1.57 * dir, Eigen::Vector3d::UnitZ());
            next_ = next_ * fwd;
            // forward
            fwd = Eigen::Translation3d(dist, 0, 0);
            forward(fwd, steps_x);

            dir *= -1;
        }

        diff = next_.inverse() * end_pose;
        // std::cout << "reaminder diff \n" << diff.translation() << "\n";
        int steps_last_x;
        int steps_last_y;
        if (dir > 0)
        {
            steps_last_x = abs(diff.translation()[0] / dist);
            steps_last_y = abs(diff.translation()[1] / dist);

            // turn
            fwd = Eigen::AngleAxisd(1.57 * dir, Eigen::Vector3d::UnitZ());
            next_ = next_ * fwd;
            // forward
            fwd = Eigen::Translation3d(dist, 0, 0);
            forward(fwd, steps_last_x);
            // turn
            fwd = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
            next_ = next_ * fwd;
            // forward
            fwd = Eigen::Translation3d(dist, 0, 0);
            forward(fwd, steps_last_y);
        }
        else
        {
            steps_last_x = abs(diff.translation()[1] / dist);
            steps_last_y = abs(diff.translation()[0] / dist);
            // turn
            fwd = Eigen::AngleAxisd(1.57 * dir, Eigen::Vector3d::UnitZ());
            next_ = next_ * fwd;
            // forward
            fwd = Eigen::Translation3d(dist, 0, 0);
            forward(fwd, steps_last_x);
            // turn
            fwd = Eigen::AngleAxisd(1.57 * dir, Eigen::Vector3d::UnitZ());
            next_ = next_ * fwd;
            // forward
            fwd = Eigen::Translation3d(dist, 0, 0);
            forward(fwd, steps_last_y);
        }

        // std::cout << steps_last_y << "\n";

        pose = geometry_msgs::PoseStamped();
        pose.pose.position.x = end_pose.translation()[0];
        pose.pose.position.y = end_pose.translation()[1];
        pose.pose.position.z = end_pose.translation()[2];
        pose.pose.orientation.x = endq.x();
        pose.pose.orientation.y = endq.y();
        pose.pose.orientation.z = endq.z();
        pose.pose.orientation.w = endq.w();

        path_.poses.push_back(pose);

        return path_;
    }
}
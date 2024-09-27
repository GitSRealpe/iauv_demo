#ifndef SCENE_
#define SCENE_

#include <string>
#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include <iauv_motion_planner/classForwards.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

namespace iauv_motion_planner
{
    IAUVPLANNER_CLASS_FORWARD(Scene);

    class Scene
    {
    private:
        ros::NodeHandle nh_;
        nav_msgs::Path path_;
        rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
        std::vector<rviz_visual_tools::colors> colorlist_ = {rviz_visual_tools::WHITE, rviz_visual_tools::BLUE};

        visualization_msgs::Marker marker_;

        ros::Timer timer;
        int counter;

        ros::Subscriber subPath;

        void drawPath(nav_msgs::Path path);
        void animatePath(const ros::TimerEvent &event);

        void pathCb(const nav_msgs::PathConstPtr &msg);

    public:
        Scene();
        ~Scene();

        ros::ServiceServer service_;
        ros::Publisher pubState;
    };

}

#endif
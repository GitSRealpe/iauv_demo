// grabs the path, the robot model, the collision box and publishes it in the planning __HAVE_SPECULATION_SAFE_VALUE

// if enabled, use rviz visual tools

// maybe also handle the map update and the collision objects

#include <iauv_motion_planner/Scene.h>
#include <shape_msgs/Mesh.h>

namespace iauv_motion_planner
{

    Scene::Scene(ros::NodeHandle &nh) : nh_(nh)
    {
        std::cout<<"scene print"<<"\n";
        std::cout << nh.getNamespace() << "\n\n";
        std::string scene_topic;
        nh.getParam("iauv_scene_topic", scene_topic);
        std::cout<<"using: "<<scene_topic<<"\n";

        visual_tools_ = std::make_shared<rviz_visual_tools::RvizVisualTools>("world_ned", scene_topic);

        counter = 0;

        marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker_.header.frame_id = "world_ned";
        marker_.mesh_resource = "package://iauv_commander/resources/g1000.dae";
        marker_.scale.x = marker_.scale.y = marker_.scale.z = 1;
        marker_.color.r = marker_.color.g = marker_.color.b = 1;
        marker_.color.a = 0.5;

        // move this timer to draw path and compute
        timer = nh_.createTimer(ros::Duration(0.1), &Scene::animatePath, this, false, false);

        subPath = nh_.subscribe("/iauv_motion_planner/path", 1, &Scene::pathCb, this);

        std::cout << "scene instace created\n";
    }

    Scene::~Scene()
    {
        std::cout << "destroying scene obj\n";
        visual_tools_->deleteAllMarkers();

        visual_tools_->trigger();
    }

    void Scene::pathCb(const nav_msgs::PathConstPtr &msg)
    {
        std::cout << "scene received path, drawing it ---->\n";
        drawPath(*msg);
    }

    void Scene::animatePath(const ros::TimerEvent &event)
    {
        if (counter < path_.poses.size())
        {
            marker_.pose = path_.poses[counter].pose;
            visual_tools_->publishMarker(marker_);
            counter++;
        }
        else
        {
            counter = 0;
        }
        visual_tools_->trigger();
    }

    void Scene::drawPath(nav_msgs::Path path)
    {
        path_ = path;
        visual_tools_->deleteAllMarkers();
        EigenSTL::vector_Vector3d puntos;
        std::vector<rviz_visual_tools::colors> colors;
        int i = 0;
        counter = 0;

        for (auto posestmp : path_.poses)
        {
            puntos.push_back({posestmp.pose.position.x, posestmp.pose.position.y, posestmp.pose.position.z});
            colors.push_back(colorlist_.at(i++ % colorlist_.size()));
            visual_tools_->publishArrow(posestmp.pose, rviz_visual_tools::RED, rviz_visual_tools::LARGE);
        }

        visual_tools_->publishPath(puntos, colors, 0.05);

        // Don't forget to trigger the publisher!
        visual_tools_->trigger();
        timer.start();
    }

}
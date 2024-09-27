#include <iauv_motion_planner/sbmp/SampleBasedPlanner.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <ros/ros.h>

// fcl stuff
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_request.h>
#include <fcl/narrowphase/collision_result.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace iauv_motion_planner
{

    SampleBasedPlanner::SampleBasedPlanner(ros::NodeHandle &nh) : Planner(nh)
    {
        std::cout << "smbp default\n";
        std::cout << nh.getNamespace() << "\n\n";
        // std::make_shared<fcl::CollisionObjectf>(this->updateMap());
        this->updateMap();
    }

    CollObjPtr SampleBasedPlanner::updateMap()
    // fcl::CollisionObjectf SampleBasedPlanner::updateMap()
    {
        // std::cout << "getting updated map from " << map_topic_ << "\n";
        // octomap_msgs::OctomapConstPtr mapa_msg = ros::topic::waitForMessage<octomap_msgs::Octomap>("/octomap_binary");
        // octomap::AbstractOcTree *abs_tree = octomap_msgs::msgToMap(*mapa_msg);
        // std::shared_ptr<octomap::OcTree> octree(dynamic_cast<octomap::OcTree *>(abs_tree));
        // std::shared_ptr<fcl::OcTreef> tree(new fcl::OcTreef(octree));
        // std::cout << tree->getFreeThres() << "\n";
        // CollGeomPtr geo(tree);
        // // everytime a new map is read, a new pointer is made
        // CollObjPtr mapColl(new fcl::CollisionObjectf(geo));
        // // mapColl.reset(new fcl::CollisionObjectf(geo));
        // // mapColl = std::make_shared<fcl::CollisionObjectf>(geo);

        octomap_msgs::OctomapConstPtr octoMsg = ros::topic::waitForMessage<octomap_msgs::Octomap>("/octomap_binary");
        std::shared_ptr<octomap::OcTree> octree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*octoMsg)));
        // convertirlo a collision object
        std::shared_ptr<fcl::OcTreef> tree(new fcl::OcTreef(octree));
        std::shared_ptr<fcl::CollisionGeometryf> geo(tree);
        mapColl = std::shared_ptr<fcl::CollisionObjectf>(new fcl::CollisionObjectf(geo));

        // mapColl->getAABB().

        // std::cout << mapColl << "\n";
        std::cout << "map obtained\n";
        return mapColl;
    }

}

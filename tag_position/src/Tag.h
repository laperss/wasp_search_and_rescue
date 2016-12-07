#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "tag_position/EditPosition.h"
#include <tf/transform_broadcaster.h>
#include <string.h>

class Tag
{
public:
    Tag();
    void Broadcast();
    int id;
    geometry_msgs::Pose2D pose;

private:
    std::string name;
    std::string name_space;
    std::string broadcast_channel;
    tf::TransformBroadcaster br;
    geometry_msgs::Pose2D position;
    tf::Transform transform;

    ros::NodeHandle n;
    bool UpdateTag(tag_position::EditPosition::Request&, tag_position::EditPosition::Response&);
    void AddTag();
    ros::ServiceServer update_tag_srv;
};

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "tag_position/EditPosition.h"
#include <tf/transform_broadcaster.h>
#include <string.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


class MovingTag
{
public:
    MovingTag();
    void Broadcast();
    int id;
    geometry_msgs::Pose2D pose;

private:
    ros::NodeHandle n;
    ros::Subscriber pos_sub;
    ros::Subscriber test_sub;

    std::string name;
    std::string name_space;

    std::string broadcast_channel;
    tf::TransformBroadcaster br;
    geometry_msgs::Pose position;
    tf::Transform transform;
    bool has_position;


    void UpdateTag(geometry_msgs::Pose pose);
    void AddTag();
    void PositionCallback(const geometry_msgs::PoseWithCovarianceStamped);

};

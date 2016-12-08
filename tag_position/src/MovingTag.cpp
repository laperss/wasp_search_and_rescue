#include "MovingTag.h"
#include "tag_position/EditPosition.h"


MovingTag::MovingTag():
    n("~"),   // use private namespace
    has_position(false),
    id(0)
{ 
    // services 
    name_space = n.getNamespace();
    n.getParam("ID", id);
    // subscriber
    pos_sub = n.subscribe("/amcl_pos", 1,&MovingTag::PositionCallback,this);
    
}

void MovingTag::AddTag()
{
    std::ostringstream ss_name, ss_channel;
    ss_name << "id_" << id;
    name = ss_name.str();
}
void MovingTag::UpdateTag(geometry_msgs::Pose pose)
{
    double x,y,z, theta;
    x = pose.position.x;
    y = pose.position.y;
    z = 0.5;    
    theta = pose.orientation.z;

    transform.setOrigin(tf::Vector3(x, y, 0));
    tf::Quaternion quat;
    quat.setRPY(0, 0,  theta);
    transform.setRotation(quat);
}

void MovingTag::Broadcast()
{
    ROS_INFO("Add tag, ID: %s", name.c_str() );
    AddTag();
    ros::Rate rate(10);
    while (ros::ok())
    {
	ros::spinOnce();
	if (has_position)
	    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
	rate.sleep();
    }
}


void MovingTag::PositionCallback(geometry_msgs::PoseWithCovarianceStamped msg)
{
    position = msg.pose.pose;
    UpdateTag(msg.pose.pose);
}

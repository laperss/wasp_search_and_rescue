#include "Tag.h"
#include "tag_position/EditPosition.h"


Tag::Tag():
    n("~"),   // use private namespace
    id(0)
{ 
    // services 
    name_space = n.getNamespace();
    n.getParam("ID", id);
    //

}

void Tag::AddTag()
{
    std::ostringstream ss_name, ss_channel;
    ss_name << "id_" << id;
    ss_channel << "tag_position/id_" << id;
    name = ss_name.str();
    update_tag_srv = n.advertiseService(ss_channel.str(), &Tag::UpdateTag, this);

    double x, y, theta;
    n.getParam("x", x);
    n.getParam("y", y);
    n.getParam("theta", theta);

    transform.setOrigin(tf::Vector3(x, y, 0));
    tf::Quaternion quat;
    quat.setRPY(0, 0,  theta);
    transform.setRotation(quat);
}
bool Tag::UpdateTag(tag_position::EditPosition::Request  &req,
		    tag_position::EditPosition::Response &res)
{
    double x,y,theta;
    x = req.pose.x;
    y = req.pose.y;
    theta = req.pose.theta;
    transform.setOrigin(tf::Vector3(x, y, 0));
    tf::Quaternion quat;
    quat.setRPY(0, 0,  theta);
    transform.setRotation(quat);

}


void Tag::Broadcast()
{
    ROS_INFO("Add tag, ID: %s", name.c_str() );
    AddTag();
    ros::Rate rate(10);
    while (ros::ok())
    {
	ros::spinOnce();
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
	rate.sleep();
    }
}

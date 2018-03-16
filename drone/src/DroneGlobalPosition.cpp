 /**
 *  This file is a part the ardrone-search-and-rescue code 
 *  and is written to work together with the tum_ardrone package.
 *  It has been merged with code from the april_tag_detection code.
 *  Linnea Persson <laperss@kth.se> (KTH Royal Institute of Technology)
 *  Code available at <https://github.com/laperss/ardrone_search_and_rescue>.
 */
#include "DroneGlobalPosition.h"
#include "HelperFunctions.h"
#include "ros/callback_queue.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sstream>
#include <string>
#include <cstdlib>
#include <map>

using namespace std;

GlobalPosition::GlobalPosition() : 
    // UPDATE VALUES FOR BOTTOM CAM??
    m_width(640),
    m_height(360),
    m_tagSize(0.14),
    m_fx(623.709),
    m_fy(582.226),
    m_px(m_width/2),
    m_py(m_height/2),
    it(n),
    mode(NONE),
    landmark_found(false)
{
    // Names of the channels
    command_channel        = n.resolveName("/tum_ardrone/com");
    dronepose_channel      = n.resolveName("/ardrone/predictedPose");
    globalpos_channel      = n.resolveName("/ardrone/global_position");
    tag_channel            = n.resolveName("/drone/observed_tags");
    bottomcam_channel      = n.resolveName("/ardrone/bottom/image_raw");
    change_camera_channel  = n.resolveName("/ardrone/togglecam");
    takephoto_channel      = n.resolveName("/drone/takephoto");

    ROS_INFO("Init GlobalPosition()");
    bottomcam_sub     = it.subscribe(bottomcam_channel, 1, &GlobalPosition::ImageCallback,this);
    ptam_sub          = n.subscribe(dronepose_channel, 1, &GlobalPosition::PositionCallback,this);

    globalpos_pub     = n.advertise<geometry_msgs::Twist>(globalpos_channel, 1);
    tags_pub          = n.advertise<drone::object_pose>(tag_channel, 1);
    toggleCam_srv     = n.serviceClient<std_srvs::Empty>(change_camera_channel);
    takephoto_srv     = n.advertiseService(takephoto_channel, &GlobalPosition::TakePhoto, this);
    change_camera_setting = n.advertiseService("drone/camera_mode", &GlobalPosition::CameraSetting, this);

}



bool GlobalPosition::CameraSetting(drone::StringService::Request& req, drone::StringService::Response&)
{   
    if (req.str == "switching")
    {
	mode = SWITCHING; 
	ROS_INFO("MODE SWITCHING");
    }
    else if(req.str == "forward")
    {
	mode = FORWARD;
	ROS_INFO("MODE FORWARD");
    }
    else if(req.str == "down")
    {
	mode = DOWN;
	ROS_INFO("MODE DOWN");
    }
}


void GlobalPosition::PositionCallback(const tum_ardrone::filter_state state)
{
    PTAM_position.linear.x = state.x;
    PTAM_position.linear.y = state.y;
    PTAM_position.linear.z = state.z;
    // State angles given in degrees!
    PTAM_position.angular.x = standardRad(state.roll*PI/180);
    PTAM_position.angular.y = standardRad(state.pitch*PI/180);
    PTAM_position.angular.z = standardRad(state.yaw*PI/180);

}
void GlobalPosition::ImageCallback(const sensor_msgs::ImageConstPtr& msg){
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::cvtColor(cv_ptr->image, image_gray, CV_BGR2GRAY);
}

void GlobalPosition::ToggleCam()
{
    toggleCam_srv.call(toggleCam_srv_srvs);
    look_up = !look_up;
}


bool GlobalPosition::TakePhoto(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
    save_next_image = true;
}

unsigned int image_count = 0;


void GlobalPosition::BroadcastPosition()
{
    static tf::TransformBroadcaster br;
    static tf::TransformBroadcaster br1;
    static tf::TransformBroadcaster br2;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(PTAM_position.linear.x - PTAM_latest_observed.linear.x, 
				    PTAM_position.linear.y - PTAM_latest_observed.linear.y, 
				    PTAM_position.linear.z - PTAM_latest_observed.linear.z));
    tf::Quaternion quat;
    quat.setRPY( standardRad(PTAM_position.angular.x - PTAM_latest_observed.angular.x), 
	         standardRad(PTAM_position.angular.y - PTAM_latest_observed.angular.y), // should y,z be removed?
		 -standardRad(PTAM_position.angular.z - PTAM_latest_observed.angular.z));
    transform.setRotation(quat);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "last_observed_tag", "drone"));

    tf::Quaternion quat_PTAM;
    tf::Transform  map_to_drone;
    quat_PTAM.setRPY(PTAM_position.angular.x, PTAM_position.angular.y, PTAM_position.angular.z);
    map_to_drone.setRotation(quat_PTAM);
    map_to_drone.setOrigin(tf::Vector3(PTAM_position.linear.x,PTAM_position.linear.y,PTAM_position.linear.z));

    br1.sendTransform(tf::StampedTransform(map_to_drone, ros::Time::now(), "PTAM_map",   "PTAM_drone"));
}

 
void GlobalPosition::Loop()
{
    ros::Time last = ros::Time::now();
    ros::Time lastLookDown = ros::Time::now();
    ros::Time lastLookUp = ros::Time::now();
    ros::Rate loop_rate(10);
    look_up = true;
    ROS_INFO("Start global positioning loop");
    //mode = SWITCHING;
    switch_cam_on = false;
    while (n.ok())
    {
	BroadcastPosition();
        
	ros::spinOnce();
	loop_rate.sleep();
	BroadcastPosition();
    }

    ROS_INFO("End global positioning loop");

}



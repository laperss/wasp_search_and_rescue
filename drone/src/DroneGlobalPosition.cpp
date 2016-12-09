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

void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
    yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
    roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}

GlobalPosition::GlobalPosition() : 
    m_tagDetector(NULL),
    m_tagCodes(AprilTags::tagCodes36h11),
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
    command_channel        = n.resolveName("tum_ardrone/com");
    dronepose_channel      = n.resolveName("ardrone/predictedPose");
    globalpos_channel      = n.resolveName("ardrone/global_position");
    tag_channel            = n.resolveName("drone/observed_tags");
    bottomcam_channel      = n.resolveName("/ardrone/bottom/image_raw");
    change_camera_channel  = n.resolveName("ardrone/togglecam");
    switchcam_on_channel   = n.resolveName("drone/switchcam_on");
    switchcam_off_channel  = n.resolveName("drone/switchcam_off");
    takephoto_channel      = n.resolveName("drone/takephoto");

    ROS_INFO("Init GlobalPosition()");
    m_tagDetector     = new AprilTags::TagDetector(m_tagCodes);
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


void GlobalPosition::UpdateLandmark(drone::object_pose state)
{
    if (landmark_found == false || latest_landmark != state.ID)
    {
	landmark_found = true;
        latest_landmark = state.ID;
	ROS_INFO("Tag %i: (%f,%f,%f)", 
		 state.ID, state.pose.linear.z, state.pose.linear.y, state.pose.linear.x);
	ROS_INFO("Update global position");
    }
    latest_observed_position = state;
    PTAM_latest_observed =  PTAM_position;

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
    ROS_INFO("Image callback");
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::cvtColor(cv_ptr->image, image_gray, CV_BGR2GRAY);
    ImageProcess(image_gray);
}

void GlobalPosition::ImageProcess(cv::Mat& image_gray) 
{
    //cv::cvtColor(image, image_gray, CV_BGR2GRAY);
    vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);
    if (save_next_image == true)
    {
	SaveImage(image_gray);
	save_next_image = false;
    }
    for (int i=0; i<detections.size(); i++) {
      GetApriltagLocation(detections[i]);
    }
}

void GlobalPosition::ToggleCam()
{
    toggleCam_srv.call(toggleCam_srv_srvs);
    look_up = !look_up;
    ROS_INFO("Look forward: %i", look_up);
}


bool GlobalPosition::TakePhoto(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
    save_next_image = true;
}

unsigned int image_count = 0;
void GlobalPosition::SaveImage(cv::Mat& image_gray)
{
    ++image_count;
    ostringstream ss;
    ss << "image_gray" << image_count << ".jpg";
    string im = ss.str();
    imwrite(im, image_gray);
}

void GlobalPosition::GetApriltagLocation(const AprilTags::TagDetection& detection) {
    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    // translation in object frame (x forward, y left,  z up)
    // rotation in camera frame    (z forward, x right, y down) 
    detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                             translation, rotation);
    Eigen::Matrix3d F;
    F <<
      1,  0,  0,
      0, -1,  0,
      0,  0,  1;
    Eigen::Matrix3d fixed_rot = F*rotation;
    double yaw, pitch, roll;
    wRo_to_euler(fixed_rot, yaw, pitch, roll);
    float orientation;
    orientation = detection.getXYOrientation();
    drone::object_pose location;
    location.ID = detection.id;
    location.pose.linear.x = translation(0);
    location.pose.linear.y = translation(1);
    location.pose.linear.z = translation(2);
    location.pose.angular.x = 0 ;
    location.pose.angular.y = 0;
    location.pose.angular.z = orientation;
    tags_pub.publish(location);
    UpdateLandmark(location);
  }

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
    //br2.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), "PTAM_drone", "PTAM_latest_tag"));
}



void GlobalPosition::BroadcastLandmark(drone::object_pose state) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(state.pose.linear.y, -state.pose.linear.z, state.pose.linear.x));

    tf::Quaternion quat;
    quat.setRPY(-state.pose.angular.x, state.pose.angular.y, -state.pose.angular.z);
    transform.setRotation(quat);
    ostringstream ss;
    ss << "id_" << state.ID;
    string name = ss.str();
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), name , "last_observed_tag"));
}
 
void GlobalPosition::Loop()
{
    ros::Time last = ros::Time::now();
    ros::Time lastLookDown = ros::Time::now();
    ros::Time lastLookUp = ros::Time::now();
    ros::Rate loop_rate(10);
    look_up = true;
    ROS_INFO("Start global positioning loop");
    mode = SWITCHING;
    switch_cam_on = false;
    while (n.ok())
    {
	ROS_INFO("Inside global position");
	BroadcastLandmark(latest_observed_position);
	BroadcastPosition();

	if (mode == SWITCHING)
	{
	    if (ros::Time::now() - lastLookDown > ros::Duration(0.6))
	    {
		ROS_INFO("Look down");
	        ToggleCam();
		ros::Duration(0.15).sleep(); // It takes time for camera to switch
		ros::spinOnce();
		ROS_INFO("Look up");
		ToggleCam();
		lastLookDown = ros::Time::now();
	    }
	    // toggleCam_srv.call(toggleCam_srv_srvs);
	    // look_up = false;
	    // ros::Duration(0.15).sleep(); // It takes time for camera to switch
	    // ros::spinOnce();
	    // ROS_INFO("Look up");
	    // toggleCam_srv.call(toggleCam_srv_srvs);
	    // look_up = true;
	    // lastLookUp = ros::Time::now();
	    // while(((ros::Time::now() - lastLookUp) < ros::Duration(0.8)) && mode == SWITCHING)
	    // {
	    // 	ros::spinOnce();
	    // 	loop_rate.sleep();
	    // 	BroadcastLandmark(latest_observed_position);
	    // 	BroadcastPosition();
	    // }
	}
	if(mode == DOWN)
	{
	    if (look_up)
		ToggleCam();
	}
	if(mode == FORWARD)
	{
	    if (!look_up)
		ToggleCam();
	}
	// else
	// {
	//     ros::spinOnce;
	//     loop_rate.sleep();
	//     BroadcastLandmark(latest_observed_position);
	//     BroadcastPosition();	
	// }
	ros::spinOnce;
	loop_rate.sleep();
	BroadcastLandmark(latest_observed_position);
	BroadcastPosition();
    }
    ROS_INFO("End global positioning loop");

}



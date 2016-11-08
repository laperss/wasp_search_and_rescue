#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"
#include "geometry_msgs/Twist.h"
#include "tum_ardrone/filter_state.h"
#include "tum_ardrone/SetCommand.h"
#include "wasp_custom_msgs/object_pose.h"
#include "HelperFunctions.h"
#include "landmark_location.h"
#include "DroneGlobalPosition.h"
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
    // UPDATE FOR BOTTOM CAM??
    m_width(640),
    m_height(360),
    m_tagSize(0.2),
    m_fx(623.709),
    m_fy(582.226),
    m_px(m_width/2),
    m_py(m_height/2),
    it(n),
    stage(NONE),
    landmark_found(false)
{
    // Names of the channels
    linnea_command_channel = n.resolveName("set_command");
    command_channel        = n.resolveName("tum_ardrone/com");
    dronepose_channel      = n.resolveName("ardrone/predictedPose");
    globalpos_channel      = n.resolveName("ardrone/global_position");
    bottomcam_channel      = n.resolveName("/ardrone/bottom/image_raw");
    togglecam_channel      = n.resolveName("ardrone/togglecam");

    ROS_INFO("Init GlobalPosition()");
    m_tagDetector     = new AprilTags::TagDetector(m_tagCodes);
    bottomcam_sub     = it.subscribe(bottomcam_channel, 1, &GlobalPosition::ImageCallback,this);
    globalpos_pub     = n.advertise<geometry_msgs::Twist>(globalpos_channel, 1);
    ptam_sub          = n.subscribe(dronepose_channel, 1, &GlobalPosition::PositionCallback,this);

    sendCommand_srv   = n.serviceClient<tum_ardrone::SetCommand>(linnea_command_channel);
    toggleCam_srv     = n.serviceClient<std_srvs::Empty>(togglecam_channel);
    setReference_srv  = n.serviceClient<tum_ardrone::SetReference>("drone_autopilot/setReference");

    int forwardTimeMS = 1;
    int downTimeMS = 0.1;
}

void GlobalPosition::UpdateLandmark(wasp_custom_msgs::object_pose state)
{
    if (landmark_found == false || latest_landmark != state.ID)
    {
	landmark_found = true;
        latest_landmark = state.ID;
	// std::cout << "Tag " << state.ID << ": ("
	// 	  << state.pose.linear.z  << ", " 
	// 	  << state.pose.linear.y << ", "
	// 	  << state.pose.linear.x << ")\n";
	// std::cout << "- Update global position\n";
    }
    // if (state.ID == 0)
    // {
    // 	current_landmark = &landmark_0;
    // }
    // else if (state.ID == 1)
    // {
    // 	current_landmark = &landmark_1;
    // }    	

    // else if (state.ID == 2)
    // {
    // 	current_landmark = &landmark_2;
    // }   

    latest_observed_position = state;
    PTAM_latest_observed =  PTAM_position;

}
void GlobalPosition::PositionCallback(const tum_ardrone::filter_state state)
{
    // State angles given in degrees!
    PTAM_position.linear.x = state.x;
    PTAM_position.linear.y = state.y;
    PTAM_position.linear.z = state.z;
    PTAM_position.angular.x = standardRad(state.roll*PI/180);
    PTAM_position.angular.y = standardRad(state.pitch*PI/180);
    PTAM_position.angular.z = standardRad(state.yaw*PI/180);

    static tf::TransformBroadcaster br;
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
    tf::Transform map_to_drone;
    quat_PTAM.setRPY(PTAM_position.angular.x, PTAM_position.angular.y, PTAM_position.angular.z);
    map_to_drone.setRotation(quat_PTAM);
    map_to_drone.setOrigin(tf::Vector3(state.x,state.y,state.z));
    br.sendTransform(tf::StampedTransform(map_to_drone, ros::Time::now(), "PTAM_map",   "PTAM_drone"));
    br.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), "PTAM_drone", "PTAM_latest_tag"));


}
void GlobalPosition::ImageCallback(const sensor_msgs::ImageConstPtr& msg){
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::cvtColor(cv_ptr->image, image_gray, CV_BGR2GRAY);
    last_image_gray = image_gray;
    ImageProcess(image_gray);
}

void GlobalPosition::ImageProcess(cv::Mat& image_gray) 
{
    //cv::cvtColor(image, image_gray, CV_BGR2GRAY);
    vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);
    for (int i=0; i<detections.size(); i++) {
      GetApriltagLocation(detections[i]);
    }
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
    wasp_custom_msgs::object_pose location;
    location.ID = detection.id;
    location.pose.linear.x = translation(0);
    location.pose.linear.y = translation(1);
    location.pose.linear.z = translation(2);
    location.pose.angular.x = 0 ;
    location.pose.angular.y = 0;
    location.pose.angular.z = orientation;
    // location.pose.angular.x = roll;
    // location.pose.angular.y = pitch;
    // location.pose.angular.z = yaw;
    // location.pose.angular.x = yaw;
    // location.pose.angular.y = roll;
    // location.pose.angular.z = pitch;

    //std::cout << "2: ("<< roll*180/PI<<", " << pitch*180/PI <<", "<< yaw*180/PI <<")\n";
    //std::cout << "o:  "<< orientation*180/PI <<"\n";
    UpdateLandmark(location);
  }

void GlobalPosition::BroadcastLandmark(wasp_custom_msgs::object_pose state) {
    //tf::StampedTransform world_to_drone;
    //global_position_listener.lookupTransform("base", "ardrone_base_bottomcam", ros::Time(0), world_to_drone);


    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(state.pose.linear.y, -state.pose.linear.z, state.pose.linear.x));

    tf::Quaternion quat;
    quat.setRPY(-state.pose.angular.x, state.pose.angular.y, -state.pose.angular.z);
    //std::cout << "ATT: ("<< state.pose.angular.x*180/PI<<", " << state.pose.angular.y*180/PI <<", "<< state.pose.angular.z*180/PI <<")\n";
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
    ros::Rate loop_rate(20);
    bool look_up = true;
    ROS_INFO("Start loop");
    while (n.ok())
    {
	toggleCam_srv.call(toggleCam_srv_srvs);
	look_up = false;
	ros::Duration(0.15).sleep(); // It takes time for camera to switch
	ros::spinOnce();
	toggleCam_srv.call(toggleCam_srv_srvs);

	look_up = true;
	lastLookUp = ros::Time::now();
	while((ros::Time::now() - lastLookUp) < ros::Duration(2.0))
	{
	    ros::spinOnce();
	    loop_rate.sleep();
	    BroadcastLandmark(latest_observed_position);
	}
    }
}



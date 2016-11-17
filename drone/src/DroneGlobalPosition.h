#include "ros/ros.h"

#include "std_srvs/Empty.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "drone/object_pose.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"
#include "tum_ardrone/filter_state.h"
#include "tum_ardrone/SetCommand.h"



class GlobalPosition
{
public:
    GlobalPosition();
    void Loop();
    static pthread_mutex_t video_bottom_thread;
private:
    enum {NONE, FORWARD, DOWN, DONE} stage;
    bool landmark_found; 
    bool landmark_visible; 
    int  latest_landmark;
    ros::NodeHandle n;

    // POSITIONING
    geometry_msgs::Twist PTAM_position;
    geometry_msgs::Twist position;
    geometry_msgs::Twist PTAM_latest_observed;  // position relative to the latest observed landmark
    drone::object_pose latest_observed_position;  // position relative to the landmark at latest observation
    
    // PUBLISHERS/SUBSCRIBERS/SERVICES
    ros::Publisher           globalpos_pub;
    ros::Publisher           tags_pub;
    ros::Subscriber          ptam_sub;

    void UpdatePosition();
    void BroadcastPosition();
    void BroadcastLandmark(drone::object_pose  state);
    void UpdateLandmark(drone::object_pose     state);
    void PositionCallback(const tum_ardrone::filter_state state);
    void ImageCallback(const sensor_msgs::ImageConstPtr&  msg);
    void ImageProcess(cv::Mat& image_gray);
    void SaveImage(cv::Mat& image_gray);
    void GetApriltagLocation(const AprilTags::TagDetection& detection);


    // LANDMARKS 
    geometry_msgs::Pose2D  landmark_0;
    geometry_msgs::Pose2D  landmark_1;
    geometry_msgs::Pose2D  landmark_2;
    geometry_msgs::Pose2D * current_landmark;
    drone::object_pose landmark; 

    AprilTags::TagDetector*          m_tagDetector;
    AprilTags::TagCodes              m_tagCodes;
    image_transport::ImageTransport  it;
    image_transport::Subscriber      bottomcam_sub;
    int    m_width;      // image size in pixels
    int    m_height;
    double m_tagSize; // April tag side length in meters of square black frame
    double m_fx;      // camera focal length in pixels
    double m_fy;
    double m_px;      // camera principal point
    double m_py;


    // ROS SERVICES, SUBSCRIBERS, PUBLISHERS
    ros::ServiceClient  toggleCam_srv;
    std_srvs::Empty     toggleCam_srv_srvs;

    // ARTAG DETECTION
    ros::Subscriber        im_sub;
    cv_bridge::CvImagePtr  cv_ptr;
    cv::Mat                image_new;
    cv::Mat                image_gray;

    // Channel names
    std::string globalpos_channel;
    std::string dronepose_channel;
    std::string command_channel;
    std::string tag_channel;
    std::string bottomcam_channel;
    std::string togglecam_channel;

};

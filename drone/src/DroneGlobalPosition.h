 /**
 *  This file is a part the ardrone-search-and-rescue code 
 *  and is written to work together with the tum_ardrone package.
 *  Linnea Persson <laperss@kth.se> (KTH Royal Institute of Technology)
 *  Code available at <https://github.com/laperss/ardrone_search_and_rescue>.
 */
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
//#include "tum_ardrone/SetCommand.h"

class GlobalPosition
{
public:
    GlobalPosition();
    void Loop();
    static pthread_mutex_t video_bottom_thread;
private:
    enum {NONE, FORWARD, DOWN, DONE} stage;
    ros::NodeHandle n;
    
    // Publishers/subscribers
    ros::Publisher                   globalpos_pub;
    ros::Publisher                   tags_pub;
    ros::Subscriber                  ptam_sub;
    image_transport::Subscriber      bottomcam_sub;

    // Services
    ros::ServiceClient  toggleCam_srv;
    std_srvs::Empty     toggleCam_srv_srvs;

    // Functions
    void UpdatePosition();
    void BroadcastPosition();
    void BroadcastLandmark(drone::object_pose  state);
    void UpdateLandmark(drone::object_pose     state);

    // Functions apriltags
    void ImageProcess(cv::Mat& image_gray);
    void SaveImage(cv::Mat& image_gray);
    void GetApriltagLocation(const AprilTags::TagDetection& detection);

    // Callbacks
    void PositionCallback(const tum_ardrone::filter_state state);
    void ImageCallback(const sensor_msgs::ImageConstPtr&  msg);

    // Variables 
    geometry_msgs::Twist PTAM_position;
    geometry_msgs::Twist PTAM_latest_observed;  // position relative to the latest observed landmark
    drone::object_pose   latest_observed_position;  // position relative to the landmark at latest observation

    bool   landmark_found; 
    bool   landmark_visible; 
    int    latest_landmark;

    // Variables  apriltags
    AprilTags::TagDetector*          m_tagDetector;
    AprilTags::TagCodes              m_tagCodes;
    image_transport::ImageTransport  it;
    int    m_width;     // image size in pixels
    int    m_height;
    double m_tagSize;   // April tag side length in meters of square black frame
    double m_fx;        // camera focal length in pixels
    double m_fy;
    double m_px;        // camera principal point
    double m_py;
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

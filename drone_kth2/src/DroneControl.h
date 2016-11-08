#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "tum_ardrone/filter_state.h"
#include "tum_ardrone/SetReference.h"
#include "tum_ardrone/SetCommand.h"
#include "ardrone_autonomy/LedAnim.h"
#include "wasp_custom_msgs/object_pose.h"
#include <tf/transform_listener.h>
#include "HelperFunctions.h"
#include "std_srvs/Empty.h"

#include <sstream>
#include <cstdlib>
#include <map>


class DroneControl
{
public:
    DroneControl();
    void Loop();
    static pthread_mutex_t video_bottom_thread;
private:
    enum {NONE, HOVER, TAG_FOLLOW, GOTO} flight_mode;
    ros::NodeHandle n;

    ros::ServiceClient         send_command_srv;
    ros::ServiceClient         set_reference_srv;
    ros::ServiceClient         set_led_anim_srv;
    ros::ServiceClient         hover_srv;

    tum_ardrone::SetReference  reference_srvs;
    tum_ardrone::SetCommand    send_command_srvs;
    std_srvs::Empty            empty_srvs;
    ardrone_autonomy::LedAnim  led_anim;

    ros::Subscriber drone_globalpos_sub;
    ros::Subscriber ptam_sub;
    ros::Publisher  tum_ardrone_pub;
    ros::Publisher  takeoff_pub;
    ros::Publisher  land_pub;
    ros::Publisher  toggleReset_pub;


    geometry_msgs::Pose2D goal;
    geometry_msgs::Twist  position;          // global position
    geometry_msgs::Twist  global_position;   // global position
    geometry_msgs::Twist  PTAM_position;     // position in PTAM frame
    std_msgs::String      tum_ardrone_msg;
    tf::TransformListener global_position_listener;
    tf::Transform  world_to_map;
    tf::Transform  map_to_drone;
    tf::Transform  map_to_world;
    void PositionCallback(const geometry_msgs::Twist state);
    void PTAMPositionCallback(const tum_ardrone::filter_state state);
    void HoverLandmark(const geometry_msgs::Pose2D);
    void GoTo(const geometry_msgs::Pose2D);
    void SetGoal(const geometry_msgs::Pose2D);

    std::string globalpos_channel;
    std::string dronepose_channel;
    std::string command_channel;
    std::string linnea_command_channel;
    std::string led_anim_channel;
    std::string send_hover_channel;
    std::string toggleReset_channel;
    std::string land_channel;
    std::string takeoff_channel;


};


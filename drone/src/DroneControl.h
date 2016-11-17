 /**
 *  This file is a part the ardrone-search-and-rescue code 
 *  and is written to work together with the tum_ardrone package.
 *  Linnea Persson <laperss@kth.se> (KTH Royal Institute of Technology)
 *  Code available at <https://github.com/laperss/ardrone_search_and_rescue>.
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "tum_ardrone/filter_state.h"
#include "tum_ardrone/SetCommand.h"
#include "ardrone_autonomy/LedAnim.h"
#include <tf/transform_listener.h>
#include "std_srvs/Empty.h"
#include <actionlib/client/simple_action_client.h>  
#include <drone/DoCommandAction.h>           

class DroneControl
{
public:
    DroneControl();
    void Loop();
    static pthread_mutex_t video_bottom_thread;
private:
    enum {NONE, HOVER, TAG_FOLLOW, GOTO} flight_mode;
    ros::NodeHandle n;

    // Services
    ros::ServiceClient         set_led_anim_srv;
    ros::ServiceClient         hover_srv;
    actionlib::SimpleActionClient<drone::DoCommandAction> drone_command;

    // Publishers/Subscribers
    ros::Subscriber drone_globalpos_sub;
    ros::Subscriber tum_ardrone_sub;
    ros::Subscriber ptam_sub;
    ros::Publisher  tum_ardrone_pub;
    ros::Publisher  takeoff_pub;
    ros::Publisher  land_pub;
    ros::Publisher  toggleReset_pub;

    // Variables
    ardrone_autonomy::LedAnim  led_anim;
    geometry_msgs::Pose2D      goal_;
    geometry_msgs::Twist       position;          // global position
    geometry_msgs::Twist       global_position;   // global position
    geometry_msgs::Twist       PTAM_position;     // position in PTAM frame
    std_msgs::String           tum_ardrone_msg;
    tf::TransformListener      global_position_listener;
    
    // Transforms
    tf::Transform         map_to_world;
    tf::StampedTransform  world_to_drone;
    tf::StampedTransform  map_to_drone;
    tf::Transform         world_to_goal;

    // Callbacks
    void PositionCallback(const geometry_msgs::Twist state);
    void PTAMPositionCallback(const tum_ardrone::filter_state state);
    void TUMArdroneCallback(const std_msgs::String s);
    void feedbackCb(const drone::DoCommandFeedbackConstPtr& feedback);
    void activeCb();
    void doneCb(const actionlib::SimpleClientGoalState& state,
			     const drone::DoCommandResultConstPtr& result);
    // Methods
    void HoverLandmark(const geometry_msgs::Pose2D);
    void LookForLandmark();
    void GoTo(const geometry_msgs::Pose2D);
    void GoToCoordinate(const geometry_msgs::Pose2D);
    void SetGoal(const geometry_msgs::Pose2D);
    void PickUp();
    void Deliver();
    void Land();
    void Takeoff();
    void ClearCommands();
    void StartServer();

    // Channel names
    std::string globalpos_channel;
    std::string dronepose_channel;
    std::string command_channel;
    std::string led_anim_channel;
    std::string send_hover_channel;
    std::string toggleReset_channel;
    std::string land_channel;
    std::string takeoff_channel;


};


 /**
 *  This file is a part the ardrone-search-and-rescue code 
 *  and is written to work together with the tum_ardrone package.
 *  Linnea Persson <laperss@kth.se> (KTH Royal Institute of Technology)
 *  Code available at <https://github.com/laperss/ardrone_search_and_rescue>.
 */
#include <vector>
#include <algorithm>
#include <map>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "drone/StringService.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "diagnostic_msgs/KeyValue.h"
#include "tum_ardrone/filter_state.h"
#include "tum_ardrone/SetMaxControl.h"
#include "drone/object_pose.h"
#include "ardrone_autonomy/LedAnim.h"
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>  
#include <actionlib/server/simple_action_server.h>  
#include <drone/DoCommandAction.h>           
#include <drone/DoPositionCommandAction.h>
#include "rosplan_action_interface/RPActionInterface.h"

typedef actionlib::SimpleActionClient<drone::DoPositionCommandAction> SendPositionCommandClient; 
typedef actionlib::SimpleActionClient<drone::DoCommandAction>         SendCommandClient; 
typedef actionlib::SimpleActionServer<drone::DoPositionCommandAction> SendPositionCommandServer; 
typedef actionlib::SimpleActionServer<drone::DoCommandAction>         SendCommandServer; 


class DroneControl
{
public:
    DroneControl();
    void Setup();
    void Loop();
    struct Command {
	int id;
	std::map<std::string, std::string> param;
	double dispatch_time;
	std::string name;
    } ;
    struct ControlCommand
    {
	inline ControlCommand() {roll = pitch = yaw = gaz = 0;}
	inline ControlCommand(double roll, double pitch, double yaw, double gaz)
	    {
		this->roll = roll;
		this->pitch = pitch;
		this->yaw = yaw;
		this->gaz = gaz;
	    }
	double yaw, roll, pitch, gaz;
};
private:
    enum {NONE, HOVER, TAG_FOLLOW, GOTO, ACTION, TAG_LOOK}  flight_mode;
    enum {AVAILABLE, BUSY, FINISHED}          action_mode;
    ros::NodeHandle n;

    // Services
    ros::ServiceClient         set_led_anim_srv;
    ros::ServiceClient         set_max_control_srv;
    ros::ServiceClient         camera_setting_srv;
    ros::ServiceClient         toggleCam_srv;

    // Action servers 
    SendCommandClient            send_command_client;
    SendPositionCommandClient    send_position_command_client;

    SendCommandServer            pickup_srv;
    SendCommandServer            deliver_srv;
    SendCommandServer            tag_follow_srv;
    SendCommandServer            land_srv;
    SendCommandServer            takeoff_srv;
    SendCommandServer            look_for_tag_srv;
    SendCommandServer            hover_srv;
    SendPositionCommandServer    autoinit_srv;
    SendPositionCommandServer    goto_srv;
    SendPositionCommandServer    moveby_srv;

    void Goto(const drone::DoPositionCommandGoalConstPtr& goal); 
    void MoveBy(const drone::DoPositionCommandGoalConstPtr& goal); 
    void AutoInit(const drone::DoPositionCommandGoalConstPtr& goal); 
    void Takeoff(const drone::DoCommandGoalConstPtr& goal); 
    void Land(const drone::DoCommandGoalConstPtr& goal); 
    void Hover(const drone::DoCommandGoalConstPtr& goal); 
    void Deliver(const drone::DoCommandGoalConstPtr& goal); 
    void PickUp(const drone::DoCommandGoalConstPtr& goal);     
    void TagFollow(const drone::DoCommandGoalConstPtr& goal);     
    void LookForTag(const drone::DoCommandGoalConstPtr& goal);     
    void SendControlToDrone(ControlCommand cmd);
	 

    // Publishers/Subscribers
    ros::Subscriber drone_globalpos_sub;
    ros::Subscriber tum_ardrone_sub;
    ros::Subscriber ptam_sub;
    ros::Subscriber tags_sub;
    ros::Publisher  tum_ardrone_pub;
    ros::Publisher  takeoff_pub;
    ros::Publisher  land_pub;
    ros::Publisher  toggleReset_pub;
    ros::Publisher  direct_cmd_pub;

    // Variables
    tum_ardrone::SetMaxControl    set_control;
    geometry_msgs::Pose2D         goal_;
    drone::object_pose            tag_relative_position;
    geometry_msgs::Twist          position;          // global position
    geometry_msgs::Twist          PTAM_position;     // position in PTAM frame
    std_msgs::String              tum_ardrone_msg;
    tf::TransformListener         global_position_listener; 
    bool                          tag_visible;
    ros::Time                     tag_time_last_seen;
    int                           tag_id_last_seen;
    int                           action_id;
    std::vector<int>              processed_commands;
  
    // Transforms
    tf::Transform         map_to_world;
    tf::StampedTransform  world_to_drone;
    tf::StampedTransform  map_to_drone;
    tf::Transform         world_to_goal;

    // Callbacks
    //   global position from global_position_node
    void PositionCallback(const geometry_msgs::Twist state);
    //   current PTAM state from the TUM node
    void PTAMPositionCallback(const tum_ardrone::filter_state state);
    //   tags currently in the view of the downfacing camera
    void TagsCallback(const drone::object_pose);

    // Functions
    bool HoverLandmark(int);
    void LookForLandmark();
    void GoTo(const geometry_msgs::Pose2D);
    void MoveByRel(const geometry_msgs::Twist);
    void GoToCoordinate(const geometry_msgs::Pose2D);
    void SetGoal(const geometry_msgs::Pose2D);
    void DoControl();
    void TagInFrame();
    void PickUp();
    void Deliver();
    void Land();
    void Takeoff();
    void Hover();
    void ClearCommands();
    void StartServer();
    void StartControl();
    void StopControl();
    void AutoInit();
    void LedAnimation();
    void SetMaxControl(double d);

    // Channel names
    std::string globalpos_channel;
    std::string ptam_channel;
    std::string command_channel;
    std::string led_anim_channel;
    std::string send_hover_channel;
    std::string toggle_reset_channel;
    std::string land_channel;
    std::string takeoff_channel;
    std::string tags_channel;
    std::string direct_cmd_channel;
    std::string camera_mode_channel;
    std::string max_ctrl_channel;
};


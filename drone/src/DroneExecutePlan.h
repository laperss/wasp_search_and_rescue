#include "ros/ros.h"
#include <vector>
#include <algorithm>
#include <map>
#include <drone/DoCommandAction.h>         
#include <drone/DoPositionCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include "rosplan_action_interface/RPActionInterface.h"
#include "std_msgs/String.h"

typedef actionlib::SimpleActionClient<drone::DoPositionCommandAction> SendPositionCommandClient; 
typedef actionlib::SimpleActionClient<drone::DoCommandAction>         SendCommandClient; 


class DroneExecutePlan
{
public:
    DroneExecutePlan();
    void Loop();

    enum action_modes{AVAILABLE, BUSY, FINISHED} action_mode;
    struct Command {
	std::string name;
	int id;
	std::map<std::string, std::string> param;
	double dispatch_time;
    } ;
private:
    ros::NodeHandle n;
    int action_id;
    std::vector<int>              processed_commands;

    // action services
    SendCommandClient            send_takeoff;
    SendCommandClient            send_land;
    SendCommandClient            send_hover;
    SendPositionCommandClient    send_goto;
    SendPositionCommandClient    send_moveby;
    SendCommandClient            send_deliver;
    SendCommandClient            send_pickup;


    drone::DoCommandGoal            goal;
    drone::DoPositionCommandGoal    position_goal;


    // funcitons
    void DoAction(const Command& command);
    void PlanCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
    void ActionStatus();

    std::string plan_sub_channel;
    std::string plan_pub_channel;

    ros::Subscriber plan_sub;
    ros::Publisher  plan_pub;
};

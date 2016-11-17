 /**
 *  This file is a part the ardrone-search-and-rescue code 
 *  and is written to work together with the tum_ardrone package.
 *  Linnea Persson <laperss@kth.se> (KTH Royal Institute of Technology)
 *  Code available at <https://github.com/laperss/ardrone_search_and_rescue>.
 */ 
#include "DroneControl.h"
#include "HelperFunctions.h"
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <iostream>
#include <string>
#include <cstdlib>
#include <map>

DroneControl::DroneControl() : 
    flight_mode(NONE),
    drone_command("drone_command", true)
{
    // Names of the channels
    led_anim_channel       = n.resolveName("ardrone/setledanimation");
    send_hover_channel     = n.resolveName("drone_autopilot/hover");
    command_channel        = n.resolveName("tum_ardrone/com");
    dronepose_channel      = n.resolveName("ardrone/predictedPose");
    globalpos_channel      = n.resolveName("ardrone/global_position");
    takeoff_channel        = n.resolveName("ardrone/takeoff");
    land_channel           = n.resolveName("ardrone/land");
    toggleReset_channel    = n.resolveName("ardrone/reset");

    // Publishers, subscribers
    tum_ardrone_pub     = n.advertise<std_msgs::String>(command_channel,50);
    takeoff_pub	        = n.advertise<std_msgs::Empty>(takeoff_channel,1);
    land_pub	        = n.advertise<std_msgs::Empty>(land_channel,1);
    toggleReset_pub     = n.advertise<std_msgs::Empty>(toggleReset_channel,1);

    ptam_sub            = n.subscribe(dronepose_channel, 1,&DroneControl::PTAMPositionCallback,this);
    drone_globalpos_sub = n.subscribe(globalpos_channel, 1,&DroneControl::PositionCallback,this);
    tum_ardrone_sub     = n.subscribe(command_channel, 1,&DroneControl::TUMArdroneCallback,this);

    // Services
    hover_srv           = n.serviceClient<std_srvs::Empty>(send_hover_channel);
    set_led_anim_srv    = n.serviceClient<ardrone_autonomy::LedAnim>(led_anim_channel);

    // WAIT FOR INITIAL POSITION
    bool has_position = false;
    while (!has_position)
    {
	try {
	    // takes some iterations before this works, why?
	    global_position_listener.lookupTransform("world", "drone", ros::Time(0), world_to_drone);
	    position.linear.x = world_to_drone.getOrigin().getX();
	    position.linear.y = world_to_drone.getOrigin().getY();
	    position.linear.z = world_to_drone.getOrigin().getZ();
	    has_position = true;
	    ROS_INFO("Drone position found");
	}
	catch (tf::TransformException ex) {
	    ROS_ERROR("%s", ex.what());
	    ros::Duration(0.5).sleep();
	}
    }


}

void DroneControl::TUMArdroneCallback(const std_msgs::String s)
{
    std::string strng = s.data;
    if (strng.substr(0,4) == "u c ")
    {
	ROS_INFO("Feedback from drone");
	int from = strng.find("Current");
	int len   = strng.find("\n",strng.find("Current")+1,2)-from;
//	std::cout <<from<<"\n-------" << strng.substr(from, len) << "\n--------"<<len<<"\n";
    }
}

void DroneControl::PositionCallback(const geometry_msgs::Twist state)
{
    position = state;
}

void DroneControl::PTAMPositionCallback(const tum_ardrone::filter_state state)
{

    PTAM_position.linear.x = state.x;
    PTAM_position.linear.y = state.y;
    PTAM_position.linear.z = state.z;
    try {
	// takes some iterations before this works, why?
	global_position_listener.lookupTransform("world", "drone", ros::Time(0), world_to_drone);
	position.linear.x = world_to_drone.getOrigin().getX();
	position.linear.y = world_to_drone.getOrigin().getY();
	position.linear.z = world_to_drone.getOrigin().getZ();
    }
    catch (tf::TransformException ex) {
	ros::Duration(0.5).sleep();
    }
    try {
	global_position_listener.lookupTransform("PTAM_map", "PTAM_drone", ros::Time(0), map_to_drone);
    }
    catch (tf::TransformException ex) {
	ros::Duration(0.5).sleep();
    }
    map_to_world = map_to_drone*world_to_drone.inverse();


}

void DroneControl::HoverLandmark(const geometry_msgs::Pose2D lm)
{
    // not implemented
}

void DroneControl::LookForLandmark()
{
    1;
}

void DroneControl::GoToCoordinate(const geometry_msgs::Pose2D lm)
{
    ROS_INFO("Go to position: %f, %f, %f", lm.x, lm.y, 1.8);
    ros::Rate loop_rate(0.5);
    //SetGoal(lm);
    tf::Vector3 diff(lm.x-position.linear.x,lm.y-position.linear.y, 1.8-position.linear.z);

    // update if there is a NaN
    while ((diff[0] != diff[0]) || (diff[1] != diff[1]))
    {
	ROS_INFO("Not a valid coordinate: %f, %f, %f", diff[0], diff[1], diff[2]);
        diff = tf::Vector3(lm.x-position.linear.x,lm.y-position.linear.y,1.8-position.linear.z);
	loop_rate.sleep();
	ros::spinOnce();
    }

    int count = 0;
    while ((std::abs(diff[0]) > 0.2) || (std::abs(diff[1]) > 0.2)|| (std::abs(diff[2]) > 0.2))
    {
	++count;
	GoTo(lm);
	loop_rate.sleep();
	ros::spinOnce();
        diff = tf::Vector3(lm.x-position.linear.x, lm.y-position.linear.y,1.8-position.linear.z);
	ros::spinOnce();
	ROS_INFO("Number calls: %i",count); // if reached correct, should only be called once.

    }
    
    ROS_INFO("Landmark reached");
	
}


void DroneControl::GoTo(const geometry_msgs::Pose2D lm)
{
    ros::spinOnce();
    drone::DoCommandGoal goal; 
    const tf::Vector3 diff = map_to_world(tf::Vector3(lm.x,lm.y,1.8));
    goal.command_id = 6;
    // Do not send an obviously faulty message
    if ((diff[0] != diff[0]) && (diff[1] != diff[1]) && (diff[2] != diff[2]))
	ROS_INFO("Not a valid coordinate: %f, %f, %f", diff[0], diff[1], diff[2]);
    else
    {
	if ((diff[0] < 5) && (diff[1] < 5) && (diff[2] < 5))
	{
	    ROS_INFO("Send command: %f, %f, %f", diff[0], diff[1], diff[2]);
	    goal.p1 = diff[0];
	    goal.p2 = diff[1];
	    goal.p3 = diff[2];
	    goal.p4 = 0;
	    drone_command.sendGoal(goal);
//(goal, boost::bind(&DroneControl::doneCb, this, _1, _2),boost::bind(&DroneControl::activeCb, this) ,boost::bind(&DroneControl::feedbackCb, this, _1));
	    drone_command.waitForResult(ros::Duration(10.0)); // should~ sec
	    ROS_INFO("Command ended");

	}
	else
	{
	    ROS_INFO("Strange coordinate: %f, %f, %f", diff[0], diff[1], diff[2]);
	}
    }
} 


void DroneControl::SetGoal(const geometry_msgs::Pose2D lm)
{
    goal_ = lm;
    static tf::TransformBroadcaster br;
    world_to_goal.setOrigin(tf::Vector3(lm.x, lm.y, 0));
    ROS_INFO("New goal: %f, %f", lm.x, lm.y);
    tf::Quaternion quat;
    quat.setRPY(0, 0, lm.theta);
    world_to_goal.setRotation(quat);
    br.sendTransform(tf::StampedTransform(world_to_goal, ros::Time::now(), "world", "goal"));
} 


void DroneControl::Land()
{
    ROS_INFO("Land");
    drone::DoCommandGoal goal;
    goal.command_id = 4;
    drone_command.sendGoal(goal);
    drone_command.waitForResult(ros::Duration(10.0)); 
}

void DroneControl::Takeoff()
{
    ROS_INFO("Takeoff");
    drone::DoCommandGoal goal;
    goal.command_id = 3;
    drone_command.sendGoal(goal);
    drone_command.waitForResult(ros::Duration(10.0)); 
}

void DroneControl::ClearCommands()
{
    ROS_INFO("Clear commands");
    drone::DoCommandGoal goal;
    goal.command_id = 0;
    drone_command.sendGoal(goal);
    drone_command.waitForResult(ros::Duration(10.0)); 
}

void DroneControl::StartServer()
{
    ROS_INFO("Waiting for action server to start.");
    drone_command.waitForServer(); // will wait for infinite time
    ROS_INFO("Action server started, ready to send goals.");
}

void DroneControl::PickUp()
{
    ROS_INFO("Pick up!");
    drone::DoCommandGoal goal;
    led_anim.request.freq = 1;
    led_anim.request.type = 4;
    led_anim.request.duration = 4;

    goal.command_id = 7;
    goal.p1 =  0;
    goal.p2 =  0;
    goal.p3 = -1; 
    goal.p4 =  0;
    drone_command.sendGoal(goal);
    set_led_anim_srv.call(led_anim);
    drone_command.waitForResult(ros::Duration(10.0)); 
    goal.p3 =  1; 
    drone_command.sendGoal(goal);
    drone_command.waitForResult(ros::Duration(10.0)); 
    ROS_INFO("Pickup finished.");
} 

void DroneControl::Deliver()
{
    ROS_INFO("Deliver!");
    drone::DoCommandGoal goal;
    led_anim.request.freq = 1;
    led_anim.request.type = 4;
    led_anim.request.duration = 4;

    goal.command_id = 7;
    goal.p1 =  0;
    goal.p2 =  0;
    goal.p3 = -1; 
    goal.p4 =  0;
    drone_command.sendGoal(goal);
    set_led_anim_srv.call(led_anim);
    drone_command.waitForResult(ros::Duration(10.0));
    goal.p3 =  1; 
    drone_command.sendGoal(goal);
    drone_command.waitForResult(ros::Duration(10.0)); 
    ROS_INFO("Delivery finished.");
} 

void DroneControl::Loop()
{

    // WAIT FOR INITIAL POSITION
    bool has_position = false;
    while (!has_position)
    {
	try {
	    // takes some iterations before this works, why?
	    global_position_listener.lookupTransform("world", "drone", ros::Time(0), world_to_drone);
	    position.linear.x = world_to_drone.getOrigin().getX();
	    position.linear.y = world_to_drone.getOrigin().getY();
	    position.linear.z = world_to_drone.getOrigin().getZ();
	    has_position = true;
	    ROS_INFO("Drone position found");
	}
	catch (tf::TransformException ex) {
	    ROS_ERROR("%s", ex.what());
	    ros::Duration(0.5).sleep();
	}
    }
    ros::Rate loop_rate(1);
    ROS_INFO("Start Search and Rescue command loop");
    geometry_msgs::Pose2D landmark;
    geometry_msgs::Pose2D landmark2;
    landmark.x = 0;
    landmark.y = 0;
    landmark.theta = 0;
    landmark2.x = 0;
    landmark2.y = 2.3;
    landmark2.theta = 0;


    StartServer();
    ClearCommands();
    Takeoff();
    GoToCoordinate(landmark2);
    PickUp();
    GoToCoordinate(landmark);
    Deliver();
    Land();

}




//Called once when the goal completes
void DroneControl::doneCb(const actionlib::SimpleClientGoalState& state,
			  const drone::DoCommandResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Answer: %i", result->succeded);
}

// Called once when the goal becomes active
void DroneControl::activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void DroneControl::feedbackCb(const drone::DoCommandFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback of length %f", feedback->percent_complete);
}

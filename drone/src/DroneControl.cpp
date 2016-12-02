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
    // Action services
    send_command_client("drone_autopilot/empty_command", true),
    send_position_command_client("drone_autopilot/position_command", true),
    goto_srv("drone/goto", boost::bind(&DroneControl::Goto, this, _1), false),
    moveby_srv("drone/moveby", boost::bind(&DroneControl::MoveBy, this, _1), false),
    land_srv("drone/land", boost::bind(&DroneControl::Land, this, _1), false),
    takeoff_srv("drone/takeoff", boost::bind(&DroneControl::Takeoff, this, _1), false),
    hover_srv("drone/pickup", boost::bind(&DroneControl::PickUp, this, _1), false),
    deliver_srv("drone/deliver", boost::bind(&DroneControl::Deliver, this, _1), false),
    pickup_srv("drone/hover", boost::bind(&DroneControl::Hover, this, _1), false),
    next_action_id(0)
{
    moveby_srv.start();
    goto_srv.start();
    hover_srv.start();
    takeoff_srv.start();
    land_srv.start();
    pickup_srv.start();
    deliver_srv.start();

    // Names of the channels
    led_anim_channel       = n.resolveName("ardrone/setledanimation");
    command_channel        = n.resolveName("tum_ardrone/com");
    dronepose_channel      = n.resolveName("ardrone/predictedPose");
    globalpos_channel      = n.resolveName("ardrone/global_position");
    takeoff_channel        = n.resolveName("ardrone/takeoff");
    land_channel           = n.resolveName("ardrone/land");
    toggleReset_channel    = n.resolveName("ardrone/reset");
    tags_channel           = n.resolveName("ardrone/observed_tags");
    plan_channel           = n.resolveName("/kcl_rosplan/action_dispatch");
    plan_pub_channel       = n.resolveName("/kcl_rosplan/action_feedback");

    // Publishers, subscribers
    tum_ardrone_pub     = n.advertise<std_msgs::String>(command_channel,50);
    takeoff_pub	        = n.advertise<std_msgs::Empty>(takeoff_channel,1);
    land_pub	        = n.advertise<std_msgs::Empty>(land_channel,1);
    plan_pub	        = n.advertise<rosplan_dispatch_msgs::ActionFeedback>(plan_pub_channel,1);
    toggleReset_pub     = n.advertise<std_msgs::Empty>(toggleReset_channel,1);

    ptam_sub            = n.subscribe(dronepose_channel, 1,&DroneControl::PTAMPositionCallback,this);
    drone_globalpos_sub = n.subscribe(globalpos_channel, 1,&DroneControl::PositionCallback,this);
    tum_ardrone_sub     = n.subscribe(command_channel, 1,&DroneControl::TUMArdroneCallback,this);
    tags_sub            = n.subscribe(tags_channel, 1, &DroneControl::TagsCallback,this);
    plan_sub            = n.subscribe(plan_channel, 1000, &DroneControl::PlanCallback,this);
    // Services
    set_led_anim_srv    = n.serviceClient<ardrone_autonomy::LedAnim>(led_anim_channel);
    set_max_control_srv = n.serviceClient<tum_ardrone::SetMaxControl>("drone_autopilot/setMaxControl");

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
	//ROS_INFO("Feedback from drone");
	int from = strng.find("Current");
	int len   = strng.find("\n",strng.find("Current")+1,2)-from;
//	std::cout <<from<<"\n-------" << strng.substr(from, len) << "\n--------"<<len<<"\n";
    }
}

void DroneControl::TagsCallback(const drone::object_pose object)
{
    tag_time_last_seen = ros::Time::now();
    tag_id_last_seen = object.ID;
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



//::::::::::::::::::::::::::: DRONE FUNCTIONS :::::::::::::::::::::::::::

void DroneControl::HoverLandmark(const geometry_msgs::Pose2D lm)
{
    // not implemented
}

void DroneControl::LookForLandmark()
{
    Hover();
    while(!tag_visible)
    {
	ros::Duration(0.1).sleep();
    }
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
    while ((std::abs(diff[0]) > 0.3) || (std::abs(diff[1]) > 0.3)|| (std::abs(diff[2]) > 0.2))
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
    drone::DoPositionCommandGoal goal; 
    const tf::Vector3 diff = map_to_world(tf::Vector3(lm.x,lm.y,1.9));
    goal.command_id = 1;
    // Do not send an obviously faulty message
    if ((diff[0] != diff[0]) && (diff[1] != diff[1]) && (diff[2] != diff[2]))
	ROS_INFO("Not a valid coordinate: %f, %f, %f", diff[0], diff[1], diff[2]);
    else
    {
	if ((diff[0] < 5) && (diff[1] < 5) && (diff[2] < 5))
	{
	    //ROS_INFO("Send command: %f, %f, %f", diff[0], diff[1], diff[2]);
	    goal.x   = diff[0];
	    goal.y   = diff[1];
	    goal.z   = diff[2];
	    goal.yaw = 0;
	    send_position_command_client.sendGoal(goal);
//(goal, boost::bind(&DroneControl::doneCb, this, _1, _2),boost::bind(&DroneControl::activeCb, this) ,boost::bind(&DroneControl::feedbackCb, this, _1));
	    send_position_command_client.waitForResult(ros::Duration(10.0)); // should not be that long
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

void DroneControl::StartControl()
{
    ROS_INFO("Start control");
    drone::DoCommandGoal goal;
    goal.command_id = 1;
    send_command_client.sendGoal(goal);
    send_command_client.waitForResult(ros::Duration(10.0)); 
}

void DroneControl::StopControl()
{
    ROS_INFO("Stop control");
    drone::DoCommandGoal goal;
    goal.command_id = 2;
    send_command_client.sendGoal(goal);
    send_command_client.waitForResult(ros::Duration(10.0)); 
}

void DroneControl::Land()
{
    ROS_INFO("Land");
    drone::DoCommandGoal goal;
    goal.command_id = 4;
    send_command_client.sendGoal(goal);
    send_command_client.waitForResult(ros::Duration(10.0)); 
}

void DroneControl::Takeoff()
{
    ROS_INFO("Takeoff");
    drone::DoCommandGoal goal;
    goal.command_id = 3;
    send_command_client.sendGoal(goal);
    send_command_client.waitForResult(ros::Duration(10.0)); 
}

void DroneControl::ClearCommands()
{
    ROS_INFO("Clear commands");
    drone::DoCommandGoal goal;
    goal.command_id = 0;
    send_command_client.sendGoal(goal);
    send_command_client.waitForResult(ros::Duration(10.0)); 
}

void DroneControl::Hover()
{
    ROS_INFO("Hover");
    drone::DoCommandGoal goal;
    goal.command_id = 5;
    send_command_client.sendGoal(goal);
    send_command_client.waitForResult(ros::Duration(10.0)); 
}


void DroneControl::StartServer()
{
    ROS_INFO("Waiting for action servers to start.");
    send_command_client.waitForServer();          // will wait for infinite time
    send_position_command_client.waitForServer(); // will wait for infinite time
    ROS_INFO("Action servers have started, ready to send goals.");
}


void DroneControl::SetMaxControl(double d)
{
    ROS_INFO("Setting maximum control to %f", d);
    set_control.request.speed = d;
    set_max_control_srv.call(set_control);
}


void DroneControl::PickUp()
{
    ROS_INFO("Pick up!");
    drone::DoPositionCommandGoal goal;
    led_anim.request.freq = 1;
    led_anim.request.type = 4;
    led_anim.request.duration = 4;
    goal.command_id = 2;
    goal.x =  0;
    goal.y =  0;
    goal.z = -1; 
    goal.yaw =  0;
    send_position_command_client.sendGoal(goal);
    set_led_anim_srv.call(led_anim);
    send_position_command_client.waitForResult(ros::Duration(10.0)); 
    goal.z =  1; 
    send_position_command_client.sendGoal(goal);
    send_position_command_client.waitForResult(ros::Duration(10.0)); 
    ROS_INFO("Pickup finished.");
} 

void DroneControl::Deliver()
{
    ROS_INFO("Deliver!");
    drone::DoPositionCommandGoal goal;
    led_anim.request.freq = 1;
    led_anim.request.type = 4;
    led_anim.request.duration = 4;
    goal.command_id = 2;
    goal.x =  0;
    goal.y =  0;
    goal.z = -1; 
    goal.yaw =  0;
    send_position_command_client.sendGoal(goal);
    set_led_anim_srv.call(led_anim);
    send_position_command_client.waitForResult(ros::Duration(10.0));
    goal.z =  1; 
    send_position_command_client.sendGoal(goal);
    send_position_command_client.waitForResult(ros::Duration(10.0)); 
    ROS_INFO("Delivery finished.");
} 


//::::::::::::::::::::::::::::::: MAIN LOOP  ::::::::::::::::::::::::::::::::

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
    ros::Rate loop_rate(10);
    ROS_INFO("Start Search and Rescue command loop");
    StartServer();
    ClearCommands();
    StartControl();
    SetMaxControl(0.5);
    ros::spin();

}



//::::::::::::::::::::::::::: GOAL EXECUTION ::::::::::::::::::::::::::

void DroneControl::Goto(const drone::DoPositionCommandGoalConstPtr& goal)
{
    geometry_msgs::Pose2D position;
    position.x = goal->x;
    position.y = goal->y;
    position.theta = goal->yaw;
    GoToCoordinate(position);    
}


void DroneControl::MoveBy(const drone::DoPositionCommandGoalConstPtr& goal)
{
    geometry_msgs::Pose2D position;
    position.x = goal->x;
    position.y = goal->y;
    position.theta = goal->yaw;
    GoToCoordinate(position);    
}

void DroneControl::Land(const drone::DoCommandGoalConstPtr& goal)
{
    drone::DoCommandResult result_;
    ROS_INFO("Drone Land");
    Land();
    land_srv.setSucceeded(result_);
}

void DroneControl::Takeoff(const drone::DoCommandGoalConstPtr& goal)
{
    drone::DoCommandResult result_;
    ROS_INFO("Drone Takeoff");
    Takeoff();
    takeoff_srv.setSucceeded(result_);
}

void DroneControl::Hover(const drone::DoCommandGoalConstPtr& goal)
{
    drone::DoCommandResult result_;
    ROS_INFO("Drone Hover");
    Hover();
    hover_srv.setSucceeded(result_);
}

void DroneControl::PickUp(const drone::DoCommandGoalConstPtr& goal)
{
    drone::DoCommandResult result_;
    ROS_INFO("Drone: Pick up");
    PickUp();
    pickup_srv.setSucceeded(result_);
}

void DroneControl::Deliver(const drone::DoCommandGoalConstPtr& goal)
{
    drone::DoCommandResult result_;
    ROS_INFO("Drone: Pick up");
    Deliver();
    deliver_srv.setSucceeded(result_);
}



void DroneControl::PlanCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) 
{
    std::string wpID;
    bool found = false;
    std::stringstream ss;
    //ss << "Name of action: " <<  msg->name;
    //ROS_INFO(ss.str());
    //double duration = msg->duration;
    //diagnostic_msgs::KeyValue parameters = msg->parameters;
    int n = msg->parameters.size();
    int id = msg->action_id;
    ROS_INFO("ROSPLAN ACTION ID: %i",id );
    ROS_INFO("NEXT ACTION ID: %i",next_action_id );
    if (id == next_action_id)
    {
	std::string key1 = msg->parameters[0].key;
	std::string val1 = msg->parameters[0].value;
	std::string key2 = msg->parameters[1].key;
	std::string val2 = msg->parameters[1].value;
	std::string key3 = msg->parameters[2].key;
	std::string val3 = msg->parameters[2].value;
	double dispatch_time = msg->dispatch_time;
	//std::string name msg->name;
	ROS_INFO("Got message, number of parameters: %i",n);
	ROS_INFO("Key: %s, Value: %s",key1.c_str(), val1.c_str());
	ROS_INFO("Key: %s, Value: %s",key2.c_str(), val2.c_str());
	ROS_INFO("Key: %s, Value: %s",key3.c_str(), val3.c_str());
	++ next_action_id;
	ros::Duration(5).sleep();
    }
    if (id == next_action_id - 1)
    {
	rosplan_dispatch_msgs::ActionFeedback fb;
	fb.action_id = msg->action_id;
	fb.status = "action achieved";
	ROS_INFO("PUBLISH FEEDBACK!!");
	plan_pub.publish(fb);
    }
}

//::::::::::::::::::::::::::: GOAL CALLBACKS :::::::::::::::::::::::::::

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

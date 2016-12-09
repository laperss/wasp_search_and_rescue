 /**
 *  This file is a part the ardrone-search-and-rescue code 
 *  and is written to work together with the tum_ardrone package.
 *  Linnea Persson <laperss@kth.se> (KTH Royal Institute of Technology)
 *  Code available at <https://github.com/laperss/ardrone_search_and_rescue>.
 */ 
#include "DroneControl.h"
#include "HelperFunctions.h"
#include <tf/transform_broadcaster.h>
#include "std_srvs/Empty.h"
#include <sstream>
#include <iostream>
#include <string>
#include <cstdlib>
#include <map>

DroneControl::DroneControl() : 
    flight_mode(NONE),
    action_mode(BUSY),
    // Action services
    send_command_client("drone_autopilot/empty_command", true),
    send_position_command_client("drone_autopilot/position_command", true),
    goto_srv("drone/goto", boost::bind(&DroneControl::Goto, this, _1), false),
    moveby_srv("drone/moveby", boost::bind(&DroneControl::MoveBy, this, _1), false),
    land_srv("drone/land", boost::bind(&DroneControl::Land, this, _1), false),
    takeoff_srv("drone/takeoff", boost::bind(&DroneControl::Takeoff, this, _1), false),
    hover_srv("drone/hover", boost::bind(&DroneControl::Hover, this, _1), false),
    deliver_srv("drone/deliver", boost::bind(&DroneControl::Deliver, this, _1), false),
    autoinit_srv("drone/autoinit", boost::bind(&DroneControl::AutoInit, this, _1), false),
    pickup_srv("drone/pickup", boost::bind(&DroneControl::PickUp, this, _1), false),
    tag_follow_srv("drone/tag_follow", boost::bind(&DroneControl::TagFollow, this, _1), false),
    look_for_tag_srv("drone/tag_look", boost::bind(&DroneControl::LookForTag, this, _1), false)
{
    moveby_srv.start();
    goto_srv.start();
    hover_srv.start();
    takeoff_srv.start();
    land_srv.start();
    pickup_srv.start();
    deliver_srv.start();
    tag_follow_srv.start();

    // Names of the channels
    led_anim_channel       = n.resolveName("ardrone/setledanimation");
    command_channel        = n.resolveName("tum_ardrone/com");
    ptam_channel           = n.resolveName("ardrone/predictedPose");
    globalpos_channel      = n.resolveName("ardrone/global_position");
    takeoff_channel        = n.resolveName("ardrone/takeoff");
    land_channel           = n.resolveName("ardrone/land");
    direct_cmd_channel     = n.resolveName("/cmd_vel");
    toggle_reset_channel   = n.resolveName("ardrone/reset");
    tags_channel           = n.resolveName("drone/observed_tags");
    max_ctrl_channel       = n.resolveName("drone_autopilot/setMaxControl");
    camera_mode_channel    = n.resolveName("drone/camera_mode");


    // Publishers, subscribers
    tum_ardrone_pub   = n.advertise<std_msgs::String>(command_channel,50);
    takeoff_pub	      = n.advertise<std_msgs::Empty>(takeoff_channel,1);
    land_pub	      = n.advertise<std_msgs::Empty>(land_channel,1);
    toggleReset_pub   = n.advertise<std_msgs::Empty>(toggle_reset_channel,1);
    direct_cmd_pub    = n.advertise<geometry_msgs::Twist>(direct_cmd_channel,1);

    ptam_sub            = n.subscribe(ptam_channel, 1,&DroneControl::PTAMPositionCallback,this);
    drone_globalpos_sub = n.subscribe(globalpos_channel, 1,&DroneControl::PositionCallback,this);
    tags_sub            = n.subscribe(tags_channel, 1, &DroneControl::TagsCallback,this);

    // Services
    set_led_anim_srv      = n.serviceClient<ardrone_autonomy::LedAnim>(led_anim_channel);
    set_max_control_srv   = n.serviceClient<tum_ardrone::SetMaxControl>(max_ctrl_channel);
    camera_setting_srv    = n.serviceClient<drone::StringService>(camera_mode_channel);
    toggleCam_srv         = n.serviceClient<std_srvs::Empty>("ardrone/togglecam");

}

void DroneControl::TagsCallback(const drone::object_pose object)
{
    tag_visible = true;
    tag_time_last_seen = ros::Time::now();
    tag_id_last_seen = object.ID;
    tag_relative_position = object;
    if (flight_mode==TAG_FOLLOW)
    {
	ControlCommand cmd;
	cmd.roll = -0.1*tag_relative_position.pose.linear.y;
	cmd.pitch = 0.1*tag_relative_position.pose.linear.z;
	cmd.gaz = (1-tag_relative_position.pose.linear.x)*0.1;
	ROS_INFO("GAZ: %f", cmd.gaz);
	SendControlToDrone(cmd);
    }

}

void DroneControl::PositionCallback(const geometry_msgs::Twist state)
{
    // Save the global position of the drone
    position = state;
}

void DroneControl::PTAMPositionCallback(const tum_ardrone::filter_state state)
{
    // Save the position in PTAM frame of reference
    PTAM_position.linear.x = state.x;
    PTAM_position.linear.y = state.y;
    PTAM_position.linear.z = state.z;
    // Map between global posision and PTAM position
    try {
	global_position_listener.lookupTransform("world", "drone", ros::Time(0), world_to_drone);
	position.linear.x = world_to_drone.getOrigin().getX();
	position.linear.y = world_to_drone.getOrigin().getY();
	position.linear.z = world_to_drone.getOrigin().getZ();
    }
    catch (tf::TransformException ex) {
	ros::Duration(0.2).sleep();
    }
    try {
	global_position_listener.lookupTransform("PTAM_map", 
						 "PTAM_drone", ros::Time(0), map_to_drone);
    }
    catch (tf::TransformException ex) {
	ros::Duration(0.2).sleep();
    }
    map_to_world = map_to_drone*world_to_drone.inverse();
}
 
//::::::::::::::::::::::::::: DRONE FUNCTIONS :::::::::::::::::::::::::::

bool DroneControl::HoverLandmark(int id)
{
    ROS_INFO("HOVER LANDMARK: %i", id);
    flight_mode = TAG_FOLLOW;
    StopControl();
    return true;
}

void DroneControl::DoControl()
{
    if (flight_mode==TAG_FOLLOW)
    {
	if (tag_visible)
	{
	    ROS_INFO("Following landmark");
	    ControlCommand cmd;
	    cmd.roll = -0.1*tag_relative_position.pose.linear.y;
	    cmd.pitch = 0.1*tag_relative_position.pose.linear.z;
	    SendControlToDrone(cmd);
	}
	else
	{
	    ROS_INFO("ERROR HOVER LANDMARK: no visible tag");
	    ControlCommand cmd;
	    cmd.roll  = 0;
	    cmd.pitch = 0;
	    SendControlToDrone(cmd);
	}
    }
    else if (flight_mode==HOVER)
    {
	    ControlCommand cmd;
	    cmd.roll  = 0;
	    cmd.pitch = 0;
	    SendControlToDrone(cmd);
    }

}

void DroneControl::SendControlToDrone(ControlCommand cmd)
{
    ROS_INFO("SEND CONTROL TO DRONE: %f, %f", cmd.roll, cmd.pitch);
    geometry_msgs::Twist cmdT;
    cmdT.angular.z = -cmd.yaw;
    cmdT.linear.z = cmd.gaz;
    cmdT.linear.x = -cmd.pitch;
    cmdT.linear.y = -cmd.roll;
    cmdT.angular.x = cmdT.angular.y = 0;
    direct_cmd_pub.publish(cmdT);
}

void DroneControl::LookForLandmark()
{
    Hover();
    while(!tag_visible)
    {
	ros::Duration(0.1).sleep();
	ros::spinOnce();
    }
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
	if ((diff[0] < 10) && (diff[1] < 10) && (diff[2] < 10))
	{
	    //ROS_INFO("Send command: %f, %f, %f", diff[0], diff[1], diff[2]);
	    goal.x   = diff[0];
	    goal.y   = diff[1];
	    goal.z   = diff[2];
	    goal.yaw = 0;
	    send_position_command_client.sendGoal(goal);
	    send_position_command_client.waitForResult(ros::Duration(10.0)); // should not be that long
	}
	else
	{
	    ROS_INFO("Strange coordinate: %f, %f, %f", diff[0], diff[1], diff[2]);
	}
    }
} 


void DroneControl::MoveByRel(const geometry_msgs::Twist lm)
{
    ROS_INFO("move by rel: %f, %f", lm.linear.x, lm.linear.y);
    drone::DoPositionCommandGoal goal; 
    goal.command_id = 2;
    if ((lm.linear.x < 5) && (lm.linear.y < 5) && (lm.linear.z < 5))
    {
	goal.x   = lm.linear.x;
	goal.y   = lm.linear.y;
	goal.z   = 0;
	goal.yaw = 0;
	send_position_command_client.sendGoal(goal);
	send_position_command_client.waitForResult(ros::Duration(10.0)); // should not be that long
	ROS_INFO("Command ended");
    }
    else
    {
	ROS_INFO("Strange coordinate: %f, %f, %f", lm.linear.x, lm.linear.y, 0.0);
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

void DroneControl::AutoInit()
{
    ROS_INFO("AutoInit.");
    drone::DoPositionCommandGoal goal;
    goal.command_id = 6;
    send_position_command_client.sendGoal(goal);
    send_position_command_client.waitForResult(ros::Duration(30.0)); 
}

void DroneControl::SetMaxControl(double d)
{
    ROS_INFO("Setting maximum control to %f", d);
    set_control.request.speed = d;
    set_max_control_srv.call(set_control);
}

void DroneControl::LedAnimation()
{
    ardrone_autonomy::LedAnim     led_anim;
    led_anim.request.freq = 1;
    led_anim.request.type = 4;
    led_anim.request.duration = 4;
    set_led_anim_srv.call(led_anim);
}

void DroneControl::PickUp()
{
    ROS_INFO("Pick up!");
    drone::DoPositionCommandGoal goal;
    goal.command_id = 2;
    goal.x = goal.y = goal.yaw = 0;
    goal.z = -1; 
    LedAnimation();
    send_position_command_client.sendGoal(goal);
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
    goal.command_id = 2;
    goal.x = goal.y = goal.yaw = 0;
    goal.z = -1; 
    send_position_command_client.sendGoal(goal);
    LedAnimation();
    send_position_command_client.waitForResult(ros::Duration(10.0));
    goal.z =  1; 
    send_position_command_client.sendGoal(goal);
    send_position_command_client.waitForResult(ros::Duration(10.0)); 
    ROS_INFO("Delivery finished.");
} 

void DroneControl::TagInFrame()
{
    if(ros::Time::now()- tag_time_last_seen < ros::Duration(0.5))
    {
	tag_visible = true;
    }
    else
	tag_visible = false;
}

//::::::::::::::::::::::::::::::: MAIN LOOP  ::::::::::::::::::::::::::::::::

void DroneControl::Setup()
{
    ROS_INFO("Start drone control setup");
    bool has_position = false;
    while (!has_position)
    {
	try {
	    global_position_listener.lookupTransform("world","drone",ros::Time(0), world_to_drone);
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
    StartServer();
    ClearCommands();
    StartControl();
    SetMaxControl(0.5);
    action_mode = AVAILABLE;
    ROS_INFO("Setup finished");

}
void DroneControl::Loop()
{
    ros::Rate loop_rate(10);
    ROS_INFO("Start Search and Rescue command loop");
    ros::Duration(0.5).sleep();
    while(ros::ok())
    {
	ros::spinOnce();
        loop_rate.sleep();
	TagInFrame();
	DoControl();
    }
}



//::::::::::::::::::::::::::: GOAL EXECUTION - ACTION SRV ::::::::::::::::::::::::::

void DroneControl::Goto(const drone::DoPositionCommandGoalConstPtr& pos)
{
    ros::Time start_time = ros::Time::now();
    drone::StringService srv;
    srv.request.str = "switching";
    camera_setting_srv.call(srv);
    geometry_msgs::Pose2D position;
    position.x = pos->x;
    position.y = pos->y;
    position.theta = pos->yaw;
    flight_mode = GOTO;
    GoTo(position);    

    ros::spinOnce();
    drone::DoPositionCommandGoal goal; 
    const tf::Vector3 diff = map_to_world(tf::Vector3(pos->x, pos->y, 1.7));
    goal.command_id = 1;
    // Do not send an obviously faulty message
    if ((diff[0] != diff[0]) && (diff[1] != diff[1]) && (diff[2] != diff[2]))
	ROS_INFO("Not a valid coordinate: %f, %f, %f", diff[0], diff[1], diff[2]);
    else if ((diff[0] < 10) && (diff[1] < 10) && (diff[2] < 10))
    {
	ROS_INFO("Send command: %f, %f, %f", diff[0], diff[1], diff[2]);
	goal.x   = diff[0];
	goal.y   = diff[1];
	goal.z   = diff[2];
	goal.yaw = 0;
	send_position_command_client.sendGoal(goal);
	send_position_command_client.waitForResult(ros::Duration(10.0)); // should not be that long
	ROS_INFO("Command ended");
    }
}
void DroneControl::MoveBy(const drone::DoPositionCommandGoalConstPtr& goal)
{
    geometry_msgs::Pose2D position;
    position.x = goal->x;
    position.y = goal->y;
    position.theta = goal->yaw;
    flight_mode = GOTO;
    //   GoToCoordinate(position);    
}

void DroneControl::AutoInit(const drone::DoPositionCommandGoalConstPtr& goal)
{
    AutoInit();    
}

void DroneControl::Land(const drone::DoCommandGoalConstPtr& goal)
{
    drone::DoCommandResult result_;
    ROS_INFO("Action Land");
    Land();
    flight_mode = ACTION;
    result_.succeded = true;
    land_srv.setSucceeded(result_);
}

void DroneControl::Takeoff(const drone::DoCommandGoalConstPtr& goal)
{
    drone::DoCommandResult result_;
    ROS_INFO("Action Takeoff");
    flight_mode = ACTION;
    Takeoff();
    result_.succeded = true;
    takeoff_srv.setSucceeded(result_);
}

void DroneControl::Hover(const drone::DoCommandGoalConstPtr& goal)
{
    drone::DoCommandResult result_;
    ROS_INFO("Action Hover");
    Hover();
    result_.succeded = true;
    hover_srv.setSucceeded(result_);
}

void DroneControl::PickUp(const drone::DoCommandGoalConstPtr& goal)
{
    drone::DoCommandResult result_;
    ROS_INFO("Action: Pick up");
    flight_mode = ACTION;
    PickUp();
    pickup_srv.setSucceeded(result_);  // This gives error: You are attempting to call methods on an uninitialized goal handle
}

void DroneControl::Deliver(const drone::DoCommandGoalConstPtr& goal)
{
    drone::DoCommandResult result_;
    ROS_INFO("Action: Deliver");
    flight_mode = ACTION;
    Deliver();
    result_.succeded = true;
    deliver_srv.setSucceeded(result_);
}

void DroneControl::TagFollow(const drone::DoCommandGoalConstPtr& goal)
{

    ROS_INFO("Action: Follow tag");
    drone::DoCommandResult result_;
    int id = goal->command_id;
    drone::StringService srv;
    srv.request.str = "down";
    camera_setting_srv.call(srv);
    ROS_INFO("Action: Follow tag %i", id);
    flight_mode = TAG_FOLLOW;
    result_.succeded = true;
    tag_follow_srv.setSucceeded(result_);
}

void DroneControl::LookForTag(const drone::DoCommandGoalConstPtr& goal)
{

    ROS_INFO("Action: Look for tag");
    drone::DoCommandResult result_;
    int id = goal->command_id;
    drone::StringService srv;
    srv.request.str = "down";
    camera_setting_srv.call(srv);
    flight_mode = TAG_LOOK;
    result_.succeded = true;
    tag_follow_srv.setSucceeded(result_);
}

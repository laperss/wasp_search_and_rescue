#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "tum_ardrone/filter_state.h"
#include "ardrone_autonomy/LedAnim.h"
#include "tum_ardrone/SetCommand.h"
#include "wasp_custom_msgs/object_pose.h"
#include "HelperFunctions.h"
#include "std_srvs/Empty.h"
#include "tum_ardrone/SetReference.h"
#include "DroneControl.h"
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
    set_reference_srv = n.serviceClient<tum_ardrone::SetCommand>("/drone_autopilot/setReference");

    // Names of the channels
    //linnea_command_channel = n.resolveName("set_command");
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
    //send_command_srv    = n.serviceClient<tum_ardrone::SetCommand>(linnea_command_channel);
    hover_srv           = n.serviceClient<std_srvs::Empty>(send_hover_channel);
    set_led_anim_srv    = n.serviceClient<ardrone_autonomy::LedAnim>(led_anim_channel);

}

void DroneControl::TUMArdroneCallback(const std_msgs::String s)
{
    std::string strng = s.data;
    if (strng.substr(0,4) == "u c ")
    {
	ROS_INFO("Feedback from drone");
	int from = strng.find("Current");
	int len   = strng.find("\n",strng.find("Current")+1,2)-from;
	std::cout <<from<<"\n-------" << strng.substr(from, len) << "\n--------"<<len<<"\n";
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
	//ROS_ERROR("%s", ex.what());
	ros::Duration(1.0).sleep();
    }
    try {
	global_position_listener.lookupTransform("PTAM_map", "PTAM_drone", ros::Time(0), map_to_drone);
    }
    catch (tf::TransformException ex) {
	//ROS_ERROR("%s", ex.what());
	ros::Duration(1.0).sleep();
    }


    world_to_map = world_to_drone*map_to_drone.inverse();
    map_to_world = map_to_drone*world_to_drone.inverse();
    drone_to_world = world_to_drone.inverse();

    // static tf::TransformBroadcaster br;
    // br.sendTransform(tf::StampedTransform(world_to_map, ros::Time::now(), "PTAMmap", "world"));


}

void DroneControl::HoverLandmark(const geometry_msgs::Pose2D lm)
{
    std::ostringstream ss;
    //tum_ardrone_msg.data = "setReference $POSE$";  // give commands relative to current position
    //ss <<  "c moveByRel " <<  -dy  << " " <<  -dx  << " " << -dz << " " << 0;
    //ss <<  "c goto " <<  lm.y  << " " <<  lm.x  << " " << 1.5 << " " << 0;
    //ss <<  "c goto " <<  -dy  << " " <<  -dx  << " " << 1.5 << " " << 0;
    const tf::Vector3 diff = world_to_map*tf::Vector3(lm.y,lm.x,0.5);
   
    ss <<  "c goto " <<  diff[0]  << " " <<  diff[1]  << " " << diff[2] << " " << 0;

    std_msgs::String msg;
    msg.data = ss.str();

    if (diff[0] < 5 && diff[1] < 5 && diff[2] < 5)
    {
	std::cout << ss.str() << "\n";
	tum_ardrone_pub.publish(msg);
    }
    else{
	std::cout <<"not a valid point!!\n";
    }
    ros::Time startSleep = ros::Time::now();
    ros::Rate loop_rate(10);
    while((ros::Time::now() - startSleep) < ros::Duration(2.0))
    {
	ros::spinOnce();
	loop_rate.sleep();
    }
}

void DroneControl::LookForLandmark()
{
    1;
}

void DroneControl::GoToCoordinate(const geometry_msgs::Pose2D lm)
{
    ROS_INFO("GOTO POSITION");
    ros::Rate loop_rate(0.5);
    SetGoal(lm);
    tf::Vector3 diff(lm.x-position.linear.x,lm.y-position.linear.y, 1.8-position.linear.z);
    std::cout <<  "diff:  " << diff[0] << " " << diff[1] << " " <<  diff[2] << "\n ";

    // update if there is a NaN
    while ((diff[0] != diff[0]) || (diff[1] != diff[1])){
        diff = tf::Vector3(lm.x-position.linear.x,lm.y-position.linear.y,1.8-position.linear.z);
	std::cout <<  "diff:  " << diff[0] << " " << diff[1] << " " <<  diff[2] << "\n ";
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
	std::cout <<  "global:  " << position.linear.x  << " " << position.linear.y  << " " << position.linear.z << "\n ";
	std::cout <<  "diff:  " << diff[0] << " " << diff[1] << " " <<  diff[2] << "\n ";
	ROS_INFO("Number calls: %i",count);
	ros::spinOnce();
    }
    
    ROS_INFO("Landmark reached");
	
}


void DroneControl::GoTo(const geometry_msgs::Pose2D lm)
{
    ros::spinOnce();
    drone::DoCommandGoal goal; 
   //std::ostringstream ss;
    //tum_ardrone_msg.data = "setReference $POSE$";  // give commands relative to current position
    const tf::Vector3 diff = map_to_world(tf::Vector3(lm.x,lm.y,1.8));

    // ss <<  "c goto " 
    //    << diff[0] << " " 
    //    << diff[1] << " " 
    //    << diff[2] << " " << 0;
    // std::cout << ss.str() << "\n";
    // std_msgs::String msg;
    // msg.data = ss.str();
    


    goal.command_id = 6;


    // Do not send an obviously faulty message
    if ((diff[0] != diff[0]) && (diff[1] != diff[1]) && (diff[2] != diff[2]))
	std::cout <<"Got a NaN\n";
    else
    {
	if ((diff[0] < 5) && (diff[1] < 5) && (diff[2] < 5))
	{
	    goal.p1 = diff[0];
	    goal.p2 = diff[1];
	    goal.p3 = 1.8;
	    goal.p4 = 0;
	    drone_command.sendGoal(goal);
	    drone_command.waitForResult(ros::Duration(10.0)); // should take some seconds
	    //std::cout << ss.str() << "\n";
	    //tum_ardrone_pub.publish(msg);
	}
	else
	{
	    std::cout <<"Probably not a valid point!!\n";
	}
    }
} 


void DroneControl::SetGoal(const geometry_msgs::Pose2D lm)
{
    goal_ = lm;
    static tf::TransformBroadcaster br;
    world_to_goal.setOrigin(tf::Vector3(lm.x, lm.y, 0));
    std::cout << "New goal: " << lm.x << ", " << lm.y << "\n";
    tf::Quaternion quat;
    quat.setRPY(0, 0, lm.theta);
    world_to_goal.setRotation(quat);
    //const tf::Transform diff = world_to_map*world_to_goal;
    br.sendTransform(tf::StampedTransform(world_to_goal, ros::Time::now(), "world", "goal"));
} 
void DroneControl::Land()
{
    ROS_INFO("* Land");
    std::string ss =  "c land";
    std_msgs::String msg;
    msg.data = ss;
    tum_ardrone_pub.publish(msg);
}


void DroneControl::ClearCommands()
{
    ROS_INFO("* Land");
    std::string ss =  "c clearCommands";
    std_msgs::String msg;
    msg.data = ss;
    tum_ardrone_pub.publish(msg);
}

void DroneControl::PickUp(const geometry_msgs::Pose2D lm)
{
    led_anim.request.freq = 1;
    led_anim.request.type = 4;
    led_anim.request.duration = 3;
    set_led_anim_srv.call(led_anim);

    std::string ss = "c moveByRel 0 0 -0.9 0";
    std_msgs::String msg;
    msg.data = ss;
    ROS_INFO("* PICK UP");
    tum_ardrone_pub.publish(msg);

    
    ss = "c moveByRel 0 0 0.9 0";
    msg.data = ss;
    tum_ardrone_pub.publish(msg);

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
    // ros::Duration(1.0);
    // led_anim.request.freq = 1;
    // led_anim.request.type = 4;
    // led_anim.request.duration = 3;
    // set_led_anim_srv.call(led_anim);


    ROS_INFO("Waiting for action server to start.");
    drone_command.waitForServer(); //will wait for infinite time
    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    drone::DoCommandGoal goal;
    goal.command_id = 3;
    drone_command.sendGoal(goal, boost::bind(&DroneControl::doneCb, this, _1, _2),boost::bind(&DroneControl::activeCb, this) ,boost::bind(&DroneControl::feedbackCb, this, _1));
    bool finished_before_timeout = drone_command.waitForResult(ros::Duration(10.0)); // will come directly

    goal.command_id = 6;
    goal.p1 = 3;
    goal.p2 = 1;
    goal.p3 = 1;
    goal.p4 = 0;
    drone_command.sendGoal(goal);//, boost::bind(&DroneControl::doneCb, this, _1, _2),boost::bind(&DroneControl::activeCb, this) ,boost::bind(&DroneControl::feedbackCb, this, _1));
    ROS_INFO("SENT COMMAND 1");
    finished_before_timeout = drone_command.waitForResult(ros::Duration(10.0)); // should take some seconds    
    // result comes when next value is chosen... first next = first
    
    goal.command_id = 6;
    goal.p1 = 1;
    goal.p2 = 2;
    goal.p3 = 2;
    goal.p4 = 0;
    drone_command.sendGoal(goal);//, boost::bind(&DroneControl::doneCb, this, _1, _2),boost::bind(&DroneControl::activeCb, this) ,boost::bind(&DroneControl::feedbackCb, this, _1));
    ROS_INFO("SENT COMMAND 2");
    finished_before_timeout = drone_command.waitForResult(ros::Duration(10.0));


    goal.command_id = 6;
    goal.p1 = 1;
    goal.p2 = 2;
    goal.p3 = 1;
    goal.p4 = 0;
    drone_command.sendGoal(goal);//, boost::bind(&DroneControl::doneCb, this, _1, _2),boost::bind(&DroneControl::activeCb, this) ,boost::bind(&DroneControl::feedbackCb, this, _1));
    ROS_INFO("SENT COMMAND 3");
    finished_before_timeout = drone_command.waitForResult(ros::Duration(10.0));


    //GoToCoordinate(landmark);

    // flight_mode = GOTO;
    // switch (flight_mode)
    // {
    // case HOVER:
    // {
    // 	std::cout << "hover\n";
    // 	//hover_srv.call(empty_srvs);
    // }
    // case GOTO:
    // 	// GoTo(landmark2);
    // 	// ros::Duration(2.0).sleep();    
    // 	// GoTo(landmark);

    // 	GoToCoordinate(landmark2);
    // 	ros::Duration(1.0).sleep();    
    // 	GoToCoordinate(landmark);
    // 	//std::string ss = "c moveByRel 0 2.2 0 0";
    // 	//std_msgs::String msg;
    // 	//msg.data = ss;
    // 	//tum_ardrone_pub.publish(msg);
    // }
    // ros::Duration(1.0).sleep();	
    // Land();

}




//Called once when the goal completes
void DroneControl::doneCb(const actionlib::SimpleClientGoalState& state,
			  const drone::DoCommandResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Answer: %i", result->succeded);
  //ros::shutdown();
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

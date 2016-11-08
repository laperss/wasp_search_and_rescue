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
    flight_mode(NONE)
{
    set_reference_srv = n.serviceClient<tum_ardrone::SetCommand>("/drone_autopilot/setReference");

    // Names of the channels
    linnea_command_channel = n.resolveName("set_command");
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
    // Services
    send_command_srv    = n.serviceClient<tum_ardrone::SetCommand>(linnea_command_channel);
    hover_srv           = n.serviceClient<std_srvs::Empty>(send_hover_channel);
    set_led_anim_srv    = n.serviceClient<ardrone_autonomy::LedAnim>(led_anim_channel);

}

void DroneControl::PositionCallback(const geometry_msgs::Twist state)
{
    position = state;
}

void DroneControl::PTAMPositionCallback(const tum_ardrone::filter_state state)
{
    tf::StampedTransform world_to_drone;
    tf::StampedTransform map_to_drone;

    global_position_listener.lookupTransform("world", "drone", ros::Time(0), world_to_drone);
    global_position_listener.lookupTransform("PTAM_map", "PTAM_drone", ros::Time(0), map_to_drone);
    std::cout << "WORLD: (" << world_to_drone.getOrigin().getX() << ", " << world_to_drone.getOrigin().getY() << ", " <<world_to_drone.getOrigin().getZ() << ")\n";
    std::cout << "MAP:   (" << map_to_drone.getOrigin().getX() << ", " << map_to_drone.getOrigin().getY() << ", " <<map_to_drone.getOrigin().getZ() << ")\n";

    world_to_map = world_to_drone*map_to_drone.inverse();
    map_to_world = map_to_drone*world_to_drone.inverse();
}

void DroneControl::HoverLandmark(const geometry_msgs::Pose2D lm)
{
    std::ostringstream ss;
    //tum_ardrone_msg.data = "setReference $POSE$";  // give commands relative to current position
    //ss <<  "c moveByRel " <<  -dy  << " " <<  -dx  << " " << -dz << " " << 0;
    //ss <<  "c goto " <<  lm.y  << " " <<  lm.x  << " " << 1.5 << " " << 0;
    //ss <<  "c goto " <<  -dy  << " " <<  -dx  << " " << 1.5 << " " << 0;
    const tf::Vector3 diff = world_to_map*tf::Vector3(lm.y,lm.x,0.5);
    // std::cout << "x: " << diff[0] << "\n";
    // std::cout << "y: " << diff[1] << "\n";
    // std::cout << "z: " << diff[2]  << "\n";

    ss <<  "c goto " <<  diff[0]  << " " <<  diff[1]  << " " << diff[2] << " " << 0;

    std_msgs::String msg;
    std::cout << ss.str() << "\n";
    msg.data = ss.str();
    tum_ardrone_pub.publish(msg);
    ros::Time startSleep = ros::Time::now();
    ros::Rate loop_rate(10);
    while((ros::Time::now() - startSleep) < ros::Duration(2.0))
    {
	ros::spinOnce();
	loop_rate.sleep();
    }
}

void DroneControl::GoTo(const geometry_msgs::Pose2D lm)
{
    std::ostringstream ss;
    //tum_ardrone_msg.data = "setReference $POSE$";  // give commands relative to current position
    //ss <<  "c moveByRel " <<  -dy  << " " <<  -dx  << " " << -dz << " " << 0;

    const tf::Vector3 diff = world_to_map*tf::Vector3(lm.y,lm.x,0.5);
    // std::cout << "x: " << diff[0] << "\n";
    // std::cout << "y: " << diff[1] << "\n";
    // std::cout << "z: " << diff[2]  << "\n";

    ss <<  "c goto " <<  diff[0]  << " " <<  diff[1]  << " " << diff[2] << " " << 0;
    //std::cout << ss.str() << "\n";
    std_msgs::String msg;
    msg.data = ss.str();
    tum_ardrone_pub.publish(msg);
    ros::Time startSleep = ros::Time::now();
    ros::Rate loop_rate(10);
    while((ros::Time::now() - startSleep) < ros::Duration(2.0))
    {
	ros::spinOnce();
	loop_rate.sleep();
    }
} 


void DroneControl::SetGoal(const geometry_msgs::Pose2D lm)
{
    goal = world_to_map*lm;
    static tf::TransformBroadcaster br;
    tf::Transform world_to_goal;
    world_to_goal.setOrigin(tf::Vector3(lm.x, lm.y, 0));
    tf::Quaternion quat;
    quat.setRPY(0, 0, lm.theta);
    world_to_goal.setRotation(quat);
    const tf::Transform diff = world_to_map*world_to_goal;
    br.sendTransform(tf::StampedTransform(world_to_goal, ros::Time::now(), "world", "goal"));

} 

void DroneControl::Loop()
{
    ros::Time lastLookUp = ros::Time::now();
    ros::Rate loop_rate(1);
    ROS_INFO("Start SAR command loop");
    geometry_msgs::Pose2D landmark;
    geometry_msgs::Pose2D landmark2;
    landmark.x = 0;
    landmark.y = 0;
    landmark.theta = 0;
    landmark2.x = 1;
    landmark2.y = 0.7;
    landmark2.theta = 0;
    ros::Duration(1.0);
    led_anim.request.freq = 1;
    led_anim.request.type = 4;
    led_anim.request.duration = 3;
    set_led_anim_srv.call(led_anim);

    while (n.ok())
    {
	// lastLookUp = ros::Time::now();

	// while((ros::Time::now() - lastLookUp) < ros::Duration(1.0))
	// {
	//     ros::spinOnce();
	//     loop_rate.sleep();
	// }
	//std::cout << "GOTO TAG 1\n";
	//HoverLandmark(landmark);
	//HoverLandmark(landmark);
	//std::cout << "\nGOTO TAG 2\n";
	//HoverLandmark(landmark2);
	//std::cout << "\nLAND\n";
	// std::ostringstream ss;
	// ss <<  "c land";
	// std_msgs::String msg;
	// msg.data = ss.str();
	//tum_ardrone_pub.publish(msg);
	flight_mode = GOTO;
	switch (flight_mode)
	{
	case HOVER:
	{
	    std::cout << "hover\n";
	    //hover_srv.call(empty_srvs);
	}
	case GOTO:
	    SetGoal(landmark);
	    GoTo(goal);
	}

    }
    //if (landmark_found)
}


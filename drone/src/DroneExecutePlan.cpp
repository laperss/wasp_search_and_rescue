#include "DroneExecutePlan.h"

DroneExecutePlan::DroneExecutePlan():
    send_takeoff("drone/takeoff", true),
    send_land("drone/land", true),
    send_hover("drone/hover", true),
    send_goto("drone/goto", true),
    send_moveby("drone/moveby", true),
    send_deliver("drone/deliver", true),
    send_pickup("drone/pickup", true)
{
    plan_sub_channel = n.resolveName("/kcl_rosplan/action_dispatch");
    plan_pub_channel = n.resolveName("/kcl_rosplan/action_feedback");

    plan_sub = n.subscribe(plan_sub_channel, 1000, &DroneExecutePlan::PlanCallback,this);
    plan_pub = n.advertise<rosplan_dispatch_msgs::ActionFeedback>(plan_pub_channel,1);

}


void DroneExecutePlan::DoAction(const Command& command)
{
    drone::DoCommandGoal            goal;

    ROS_INFO("Do Action  %s", command.name.c_str());
    // DO ACTION
    action_mode == BUSY;
    if (command.name.compare("flytowaypoint"))
    {
	send_goto.sendGoal(position_goal);
	ROS_INFO("Goto");
	send_goto.waitForResult(ros::Duration(20.0));
    }
    else if (command.name.compare("deliver"))
    {
	send_deliver.sendGoal(goal);
	send_deliver.waitForResult(ros::Duration(20.0));
    }
    else if (command.name.compare("pickup"))
    {
	send_pickup.sendGoal(goal);
	send_pickup.waitForResult(ros::Duration(20.0));
    }
    else if (command.name.compare("takeoff"))
    {
	send_takeoff.sendGoal(goal);
	send_takeoff.waitForResult(ros::Duration(20.0));
    }
    else if (command.name.compare("land"))
    {
	send_land.sendGoal(goal);
	send_land.waitForResult(ros::Duration(20.0));
    }
    ROS_INFO("DoAction finished");
    action_mode = FINISHED;
}


void DroneExecutePlan::PlanCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) 
{
    action_id = msg->action_id;
    ROS_INFO("ROSPLAN ACTION ID: %i",action_id );
    if (action_mode == AVAILABLE)
    {
	// if this is a command not yet processed
	if (std::find(processed_commands.begin(),processed_commands.end(),action_id) == processed_commands.end())
	{
	    processed_commands.push_back(action_id);
	    Command command;
	    command.id = action_id;
	    command.dispatch_time = msg->dispatch_time;
	    command.name =  msg->name;
	    for (int i = 0; i <  msg->parameters.size(); i++)
	    {
		std::cout << i;
		std::string key = msg->parameters[i].key;
		std::string val = msg->parameters[i].value;
		command.param.insert(std::pair<std::string, std::string>(key, val));
	    }
	    DoAction(command);
	}
    } 
}


void DroneExecutePlan::ActionStatus()
{
    switch(action_mode)
    {
    case FINISHED:
    {
	ROS_INFO("PUBLISH FEEDBACK (status)!!");
	rosplan_dispatch_msgs::ActionFeedback fb;
	fb.action_id = action_id;
	fb.status = "action achieved";
	plan_pub.publish(fb);
	action_mode = AVAILABLE;
    }  
    }
}

void DroneExecutePlan::Loop()
{

  send_takeoff.waitForServer();
  ROS_INFO("Takeoff service found");
  send_land.waitForServer();
  ROS_INFO("Land service found");
  send_hover.waitForServer();
  ROS_INFO("Hover service found");
  send_goto.waitForServer();
  ROS_INFO("Goto service found");
  send_moveby.waitForServer();
  ROS_INFO("Moveby service found");


  ROS_INFO("End search and rescue");
    action_mode = AVAILABLE;
    ros::Rate rate(30);
    while (ros::ok())
    {	
	ros::spinOnce();
	ActionStatus();
	rate.sleep();
    }
}

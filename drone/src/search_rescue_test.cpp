 /**
 *  This file is a part the ardrone-search-and-rescue code 
 *  and is written to work together with the tum_ardrone package.
 *  Linnea Persson <laperss@kth.se> (KTH Royal Institute of Technology)
 *  Code available at <https://github.com/laperss/ardrone_search_and_rescue>.
 *  This is an example file, showing the use of the action services.
 */
#include "ros/ros.h"
#include <drone/DoCommandAction.h>           
#include <drone/DoPositionCommandAction.h>
#include <actionlib/client/simple_action_client.h>  
#include <actionlib/server/simple_action_server.h>  


typedef actionlib::SimpleActionClient<drone::DoPositionCommandAction> SendPositionCommandClient; 
typedef actionlib::SimpleActionClient<drone::DoCommandAction>         SendCommandClient; 


int main(int argc, char **argv)
{
  ros::init(argc, argv, "search_and_rescue_test");
  SendCommandClient               send_takeoff("drone/takeoff", true);
  SendCommandClient               send_land("drone/land", true);
  SendCommandClient               send_hover("drone/hover", true);
  SendPositionCommandClient       send_goto("drone/goto", true);
  SendPositionCommandClient       send_moveby("drone/moveby", true);
  SendCommandClient               send_deliver("drone/deliver", true);
  SendCommandClient               send_pickup("drone/pickup", true);
  SendCommandClient               send_tag_follow("drone/tag_follow", true);


  drone::DoCommandGoal            goal;
  drone::DoPositionCommandGoal    pos0,pos2,pos4,pos1,pos6,pos9, pos10;

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
  send_pickup.waitForServer();
   ROS_INFO("Pickup service found");

  pos1.x = -1.13;
  pos1.y = 1.0;
  pos1.z = 1.5;
  pos1.yaw = 0;

  pos9.x = 0;
  pos9.y = 0;
  pos9.z = 1.5;
  pos9.yaw = 0;

  pos2.x = -0.88;
  pos2.y = 4.39;
  pos2.z = 1.5;
  pos2.yaw = 0;

  pos4.x = 0.30;
  pos4.y = 5.06;
  pos4.z = 1.5;
  pos4.yaw = 0;

  pos6.x = -1.86;
  pos6.y = 4.72;
  pos6.z = 1.5;
  pos6.yaw = 0;

  send_goto.sendGoal(pos9);
  ROS_INFO("Send goto 9");
  send_goto.waitForResult(ros::Duration(30.0));
  goal.command_id = 9;
  ROS_INFO("Tag Follow 9");
  send_tag_follow.sendGoal(goal);
  send_tag_follow.waitForResult(ros::Duration(20.0));

  ros::Duration(1).sleep();

  // ROS_INFO("Pickup");
  // send_deliver.sendGoal(goal);
  // send_deliver.waitForResult(ros::Duration(20.0));

  send_goto.sendGoal(pos1);
  ROS_INFO("Send goto 1");
  send_goto.waitForResult(ros::Duration(30.0));

  goal.command_id = 1;
  ROS_INFO("Tag Follow 1");
  send_tag_follow.sendGoal(goal);
  send_tag_follow.waitForResult(ros::Duration(20.0));

  ros::Duration(1).sleep();

  send_deliver.sendGoal(goal);
  ROS_INFO("Deliver");
  send_deliver.waitForResult(ros::Duration(20.0));



  send_goto.sendGoal(pos9);
  ROS_INFO("Send goto 9");
  send_goto.waitForResult(ros::Duration(20.0));

  goal.command_id = 9;
  ROS_INFO("Tag Follow 9");
  send_tag_follow.sendGoal(goal);
  send_tag_follow.waitForResult(ros::Duration(20.0));
  ros::Duration(1).sleep();
  send_deliver.sendGoal(goal);
  ROS_INFO("Deliver");
  send_deliver.waitForResult(ros::Duration(20.0));



  send_land.sendGoal(goal);
  ROS_INFO("End search and rescue");
  return 0;
}

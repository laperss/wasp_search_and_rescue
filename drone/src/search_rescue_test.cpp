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


  drone::DoCommandGoal            goal;
  drone::DoPositionCommandGoal    position1;
  drone::DoPositionCommandGoal    position2;
  drone::DoPositionCommandGoal    position3;

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

  position1.x = 0;
  position1.y = 0;
  position1.z = 1.8;
  position1.yaw = 0;

  position2.x = 0;
  position2.y = 2;
  position2.z = 1.8;
  position2.yaw = 0;

  position3.x = 1.3;
  position3.y = 2;
  position3.z = 1.8;
  position3.yaw = 0;  

  send_goto.sendGoal(position1);
  ROS_INFO("Send goto");
  send_goto.waitForResult(ros::Duration(20.0));
  
  send_goto.sendGoal(position2);
  ROS_INFO("Goto");
  send_goto.waitForResult(ros::Duration(20.0));

  send_pickup.sendGoal(goal);
  ROS_INFO("Pickup");
  send_pickup.waitForResult(ros::Duration(20.0));

  send_goto.sendGoal(position3);
  ROS_INFO("Goto");
  send_goto.waitForResult(ros::Duration(20.0));

  send_deliver.sendGoal(goal);
  ROS_INFO("Deliver");
  send_deliver.waitForResult(ros::Duration(20.0));


  send_goto.sendGoal(position2);
  ROS_INFO("Goto");
  send_goto.waitForResult(ros::Duration(20.0));

  send_pickup.sendGoal(goal);
  ROS_INFO("Pickup");
  send_pickup.waitForResult(ros::Duration(20.0));

  send_goto.sendGoal(position3);
  ROS_INFO("Goto");
  send_goto.waitForResult(ros::Duration(20.0));

  send_deliver.sendGoal(goal);
  ROS_INFO("Deliver");
  send_deliver.waitForResult(ros::Duration(20.0));

  send_goto.sendGoal(position1);
  ROS_INFO("Send goto");
  send_goto.waitForResult(ros::Duration(20.0));


  send_land.sendGoal(goal);
  ROS_INFO("End search and rescue");
  return 0;
}

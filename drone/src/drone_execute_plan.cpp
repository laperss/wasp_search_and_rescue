 /**
 *  This file is a part the ardrone-search-and-rescue code 
 *  and is written to work together with the tum_ardrone package.
 *  Linnea Persson <laperss@kth.se> (KTH Royal Institute of Technology)
 *  Code available at <https://github.com/laperss/ardrone_search_and_rescue>.
 */
#include "ros/ros.h"
#include "DroneExecutePlan.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drone_execute_plan");
  DroneExecutePlan execute_plan;
  execute_plan.Loop();
  return 0;
}

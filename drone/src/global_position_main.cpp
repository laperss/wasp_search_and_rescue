#include "ros/ros.h"
#include "DroneGlobalPosition.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drone_position");
  GlobalPosition global_position;
  global_position.Loop();
  return 0;
}

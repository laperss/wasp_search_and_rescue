#include "ros/ros.h"
#include "DroneControl.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drone_control");
  DroneControl drone_control;
  drone_control.Loop();
  return 0;
}

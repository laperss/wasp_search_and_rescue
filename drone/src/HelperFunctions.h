#pragma once
#ifndef __HELPERFUNCTIONS_H
#define __HELPERFUNCTIONS_H
 
 
#include <stdlib.h>
#include "ros/ros.h"
#include <sys/time.h>
#include <cmath>

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;



extern unsigned int ros_header_timestamp_base ;
// gets a relative ms-time from a ROS time.
// can only be used to compute time differences, as it has no absolute reference.
inline static int getMS(ros::Time stamp = ros::Time::now())
{
	if(ros_header_timestamp_base == 0)
	{
		ros_header_timestamp_base = stamp.sec;
		std::cout << "set ts base to " << ros_header_timestamp_base << std::endl;
	}
	int mss = (stamp.sec - ros_header_timestamp_base) * 1000 + stamp.nsec/1000000;

	if(mss < 0)
		std::cout << "ERROR: negative timestamp..."<< std::endl;
	return mss;
}

inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}

inline double tic() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}

#endif /* __HELPERFUNCTIONS_H */

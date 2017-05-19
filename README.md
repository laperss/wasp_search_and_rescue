# Search and Rescue Automation
This code was written for solving an autonomous search and rescue senario as a part of a course in autonomous systems. 
The main code is found in the [drone package](https://github.com/laperss/wasp_search_and_rescue/tree/master/drone). 
The remaining packages contains code for 
[running the PDDL planner](https://github.com/laperss/wasp_search_and_rescue/tree/master/planning) 
and [adding apriltag position broadcasters](https://github.com/laperss/wasp_search_and_rescue/tree/master/tag_position).


### Implementaiton
This code has been successfully run on Ubuntu 14.04 running [ROS Indigo](http://wiki.ros.org/indigo). 
The hardware consisted of a Tutlebot 2 and a Parrot AR.Drone. 
The experiments were done in a room with Apriltags taped to the floor. 

### Dependencies
- The version of TUM AR.Drone found [here](https://github.com/laperss/tum_ardrone) 
- [ROSPLan](https://github.com/KCL-Planning/ROSPlan)
- [ROS Actionlib](https://github.com/ros/actionlib)
- [ROS Apriltags](https://github.com/laperss/apriltags)

### Video demonstration
A video demonstrating the implementation:

[![Search and Rescue Video](https://img.youtube.com/vi/BVbvRh_gY-0/0.jpg)](https://www.youtube.com/watch?v=BVbvRh_gY-0) 


## Drone Package
The drone package has three parts; the positioning, the controller, and the planner interface. 
The positioning keeps track of the position of the UAV in a global frame of reference. 
It is realized using a combination of PTAM and Apriltag localization.
The controller defines a set of services for the drone. 
The planner interface calls the controller functions and interfaces with ROSPlan and PDDL.

### [Global positioning node](https://github.com/laperss/wasp_search_and_rescue/blob/master/drone/src/DroneGlobalPosition.h)

This node updates the estimate of the current position. This is done in two ways: 

1. Parallel tracking and mapping (PTAM) through the TUM package. 
2. Position updates from known position, e.g., apriltags that the drone observes. 

The coordinate transformations are done using ROS tf broadcasters. 

### [Control node](https://github.com/laperss/wasp_search_and_rescue/blob/master/drone/src/DroneControl.h)

This node defines a set of services that controls the drone. Services include: 
- Go to coordinate (x,y,z,θ)
- Move relative (x,y,z,θ)
- Pickup(x,y)
- Deliver(x,y)
- Takeoff
- Land
- Hover
- Take photo

To be able to call functions and keep track of their progress, the action service library is used. This way, information is returned about the progress of the action. 

For the “go to coordinate” and “move relative to” functions, I used that this was already defined in the TUM package. 
However, it is not written as a ROS node that can be called, so I forked the TUM repository and rewrote parts of the code to be able to access these functions directly.
This forked version, required to run the package, can be found at: https://github.com/laperss/tum_ardrone

### [Planner interface  node](https://github.com/laperss/wasp_search_and_rescue/blob/master/drone/src/DroneExecutePlan.h)

ROSplan was used for the planner system, and in order to use the control nodes together with the AR.Drone an interface node was required.
ROSPlan sends out one command at a time, and it needs feedback when an action is complete in order to send out the next action. 
This was achieved using the actionlib package. This node was successfully implemented with a simple drone planner, which can be found [here](wasp_search_and_rescue/planning/scripts/add_knowledge_linnea.py).

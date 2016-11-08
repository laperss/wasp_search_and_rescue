Group #2 KTH AS course Autumn 2016 
==================================

Group Members
-------------
* Amir Roozbeh amirrsk@kth.se
* Dirk van Dooren dirkvd@kth.se
* Hakan Carlsson hakcar@kth.se
* Takuya Iwaki takya@kth.se
* Ludde Wessén jlwessen@kth.se
* Linnea Persson laperss@kth.se

## Requirements
1. It should support multiple ground robots and multiple drones. 
2. For the demonstration a single drone should:
  1. "pick up" at least 2 or more "boxes" of supplies, 
  2. deliver them to a single ground robot ("pick up" action two rounds or more)
  3. The ground robot transports them to a location close to the final delivery points. 
3. The ground robots should be used for long distance transportation of the boxes. (clarification about what long distance is)
4. The drones should be used for short distance transportation:
  1. taking a (virtual) box from the ground robot to its destination 
  2. taking a (virtual) box from the destination to the ground robot (the other way around). 
5. The ground robots should avoid obstacles if they are in the map. 
6. The solution should integrate planning, control, and decision making. For example, 
  1. plan how to achieve the overall mission, 
  2. plan a path from A to B avoiding obstacles and 
  3. control the robots to follow the trajectory, and 
  4. decide where the robots should rendezvous.

### Assumptions
1. You may assume that the position of all robots are known at the start
2. You may not assume that the position is known during the flight, this should be estimated. 
3. You may assume that you have a map of the environment which includes: 
  1. the location of the supplies (box)
  2. the location of the victims and 
  3. all obstacles in the environment.
4. You may assume that the boxes are clearly marked with April Tags or similar.

### Secondary requirements

Requirements we think would be nice to fulfil, but is not essential. Could be included if time allows.

1. Optimze the operation with respect to time
2. Optimze the operation with respect to fuel consumption


### Questions

1. How would the positioning work without the positioning module.

### Draft Milestone Goals/finer requirements
What part goals/milestones should we complete before we fulfil all the requirements.

* GR = Ground Robot
* FD = Flying drone

#### Initial steps

1. Get a simulation enironment 
2. Find a work methodology how we deal with code

#### Flying drone (Linnea, Dirk, Hakan)

1. Control velocity and direction of FD
2. Move FD from point A to B, along a specified trajectory
  
#### Ground robots (Ludde, Hamir, Takaya)

1. Control velocity and direction of GR
2. Move GR from point A to B, along a specified trajectory
3. Move GR from point A to B along a specified trajectory while avoiding predefined obstacles

#### Integration milestone (when FD and GR are done)

1. Make FDs load GR with boxes while avoiding each other?
2. Simplest solution to overal problem (1 FD and 1 GD):
  1. FDs load GR
  2. GR drive to middle point of targets
  3. FDs deliver from GR to targets
  4. Manually specified trajectories 
3. Second simplest solution, 2 FD and 1 GR
4. Solution with generated trajectories from a set of points and a map
5. Everything works according to requirements :)

### Secondary milestones

1. Optimize point 5 with respect to time
2. Optimize point 5 with respect to fuel consumption 
3. Make FDs to avoid each other during trajectories (could be replaced by intelligent path planning) 

Make the robots perfrom the same tasks in real world.

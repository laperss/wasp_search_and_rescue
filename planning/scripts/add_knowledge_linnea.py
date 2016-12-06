#! /usr/bin/env python
from pddl_functions import *

supplies = ['water', 'food', 'medicine']


def add_knowledge():
    # Add items
    for item in supplies:
    	add_instance('supply', item)

    humans = [Human('linnea', (3,1)), Human('batman',(2,2))]
    crates = [Crate('crate1','food',(1,0)),Crate('crate2', 'water')]

    drone  = Drone('ardrone', (0,0))
    robot  = Robot('turtlebot', (0,0))


    # Add waypoints
    connecting_distance = 1.42
    path = os.path.dirname(os.path.abspath(__file__))
    waypoints = load_waypoints(path + '/waypoints.txt')
    xdim = 3;
    ydim = 3;
    z = 0;
    j = 0;
    for x in np.linspace(-xdim,xdim,6):
        i = 0;
        for y in np.linspace(-ydim,ydim,6):
            pos = [x, y, z]
            name = 'wp_' + str(i) + '_' + str(j)
            add_instance('waypoint', name)
            add_waypoint(name, pos, connecting_distance)
            i = i + 1;
        j = j+1

    # GOALS
    humans[1].needs("water")
    humans[0].needs("food")
    rospy.sleep(1)

if __name__ == '__main__':
    start_ppdl_node()
    add_knowledge()






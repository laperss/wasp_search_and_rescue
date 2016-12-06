#! /usr/bin/env python
import os
import rospy
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import KeyValue
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
from mongodb_store.message_store import MessageStoreProxy
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
import numpy as np



# Add instance to knowledge base
def add_instance(type, name):
    global query_kb, update_kb

    instances = query_kb(type).instances
    if (name not in instances):
        try:
            new = KnowledgeItem()
            new.knowledge_type = KnowledgeItem.INSTANCE
            new.instance_type = type
            new.instance_name = name
            resp = update_kb(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, new)
            print('Adding instance [%s] to the knowledge base...'%name)
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)
    else:
        print('Instance [%s] already in the knowledge base...'%name)

# Add facts to knowledge base
def add_fact(name, values):
    global query_kb, update_kb

    try:
        new = KnowledgeItem()
        new.knowledge_type = KnowledgeItem.FACT
        new.attribute_name = name
        for key, value in values.iteritems():
            new.values.append(KeyValue(key, value))
        resp = update_kb(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, new)
        print('Adding fact [%s] to the knowledge base...'%name)
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

# Remove facts from knowledge base
def remove_fact(name, values):
    global query_kb, update_kb

    try:
        new = KnowledgeItem()
        new.knowledge_type = KnowledgeItem.FACT
        new.attribute_name = name
        for key, value in values.iteritems():
            new.values.append(KeyValue(key, value))
        resp = update_kb(KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE, new)
        print('Removing fact [%s] from the knowledge base...'%name)
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

# Add goal to knowledge base
def add_goal(name, values):
    global query_kb, update_kb

    try:
        new = KnowledgeItem()
        new.knowledge_type = KnowledgeItem.FACT
        new.attribute_name = name
        for key, value in values.iteritems():
            new.values.append(KeyValue(key, value))
        resp = update_kb(KnowledgeUpdateServiceRequest.ADD_GOAL, new)
        print('Adding goal [%s] to the knowledge base...'%name)
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

# Load waypoints from file
def load_waypoints(filename):
    waypoint_array = dict()
    for line in open(filename, 'r'):
        wp_id = line.split('[')[0]
        wp_position_string = line.split('[')[1].split(']')[0]
        waypoint_array[wp_id] = map(float, wp_position_string.split(','))

    return waypoint_array

def add_waypoint(name, pos, connecting_distance):
    global update_map
    
    try:
        newpose = PoseStamped()
        newpose.header.frame_id = 'map'
        newpose.pose.position.x = pos[0]
        newpose.pose.position.y = pos[1]
        newpose.pose.position.z = pos[2]
        newpose.pose.orientation.x = 0.0;
        newpose.pose.orientation.y = 0.0;
        newpose.pose.orientation.z = 0.0;
        newpose.pose.orientation.w = 1.0;
        occupancy_threshold = 2;

        resp = update_map(name, newpose, connecting_distance,occupancy_threshold)
        print('Adding waypoint [%s] to the knowledge base...'%name)
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

def start_ppdl_node():
    global query_kb, update_kb, update_map
    global location_marker_pub, marker_id
    rospy.init_node('add_knowledge')

    # Wait for services to start
    print('Waiting for services to start...')
    rospy.wait_for_service('/kcl_rosplan/get_current_instances')
    rospy.wait_for_service('/kcl_rosplan/update_knowledge_base')
    rospy.wait_for_service('/kcl_rosplan/roadmap_server/add_waypoint')

    # Subscribe to services
    query_kb = rospy.ServiceProxy('/kcl_rosplan/get_current_instances', GetInstanceService)
    update_kb = rospy.ServiceProxy('/kcl_rosplan/update_knowledge_base', KnowledgeUpdateService)
    update_map = rospy.ServiceProxy('/kcl_rosplan/roadmap_server/add_waypoint', AddWaypoint)
    location_marker_pub = rospy.Publisher("/kcl_rosplan/viz/waypoints", MarkerArray, queue_size = 1)


class Human:
    def __init__(self,name, position = (0,0)):
        self.type = 'human'
        self.name = name
        self.position = 'wp_' + str(position[0]) + '_'+ str(position[1])
        self.need_supplies = []
        add_instance(self.type,self.name)
        add_fact('human_at',{self.type:self.name, 'wp':self.position})
    def needs(self,supply):
        self.need_supplies.append(supply)
        add_goal('supply_at_human', {'supply':supply,'human':self.name})
    def add(self):
        add_instance(self.type,self.name)
    def update_position(self,position):
        remove_fact('human_at',{self.type:self.name,'wp':self.position})
        self.position = 'wp_' + str(position[0]) + '_'+ str(position[1])
        add_fact('human_at',{self.type:self.name,'wp':self.position})


class Crate:
    def __init__(self, name, content, position = (0,0)):
        self.type = 'crate'
        self.name = name
        self.position = 'wp_' + str(position[0]) + '_'+ str(position[1])
        self.content = content
        add_instance(self.type,self.name)
        add_fact('crate_at',{self.type:self.name,'wp':self.position})
        add_fact('crate_contains',{self.type:self.name,'supply':self.content})
    def add(self):
        add_instance(self.type,self.name)
    def update_position(self,position):
        remove_fact('crate_at',{self.type:self.name,'wp':self.position})
        self.position = 'wp_' + str(position[0]) + '_'+ str(position[1])
        add_fact('crate_at',{self.type:self.name,'wp':self.position})


class Drone:
    def __init__(self, name, position = (0,0)):
        self.type = 'drone'
        self.name = name
        self.position = 'wp_' + str(position[0]) + '_'+ str(position[1])
        add_instance(self.type,self.name)
        add_fact('drone_at',{self.type:self.name,'wp':self.position})
        add_fact('has_no_crate',{self.type:self.name})
    def add(self):
        add_instance(self.type,self.name)
    def update_position(self,position):
        remove_fact('drone_at',{self.type:self.name,'wp':self.position})
        self.position = 'wp_' + str(position[0]) + '_'+ str(position[1])
        add_fact('drone_at',{self.type:self.name,'wp':self.position})


class Robot:
    def __init__(self, name, position = (0,0)):
        self.type = 'robot'
        self.name = name
        self.position = 'wp_' + str(position[0]) + '_'+ str(position[1])
        add_instance(self.type,self.name)
        add_fact('robot_at',{'v':self.name,'wp':self.position})
    def add(self):
        add_instance(self.type,self.name)
    def update_position(self,position):
        remove_fact('robot_at',{'v':self.name,'wp':self.position})
        self.position = 'wp_' + str(position[0]) + '_'+ str(position[1])
        add_fact('robot_at',{'v':self.name,'wp':self.position})



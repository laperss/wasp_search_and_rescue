#!/usr/bin/env python  
import rospy
import roslib
import sys
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('apriltag_broadcaster')
    name = "id_" + str(rospy.get_param('~ID'))
    position = rospy.get_param('~position')
    position = position.split()
    br = tf.TransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "world"
    static_transformStamped.child_frame_id = name
    
    trans = (float(position[0]), float(position[1]), 0)

    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(position[2]))
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform(trans, quat, rospy.Time.now(), name, "world")
        rate.sleep()

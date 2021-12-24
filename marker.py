#!/usr/bin/env python

import sys
import rospy
import time
from visualization_msgs.msg import MarkerArray, Marker

pi = 3.14159
j6 = 2.57436

sys.path.insert(1, "/home/vaco/catkin_ws/src/niryo_one_python_api")

from niryo_one_python_api.niryo_one_api import *  # type: ignore

rospy.init_node("niryo_one_example_python_api")

topic = "visualization_marker"
publisher = rospy.Publisher(topic, MarkerArray)

rospy.init_node("register")

markerArray = MarkerArray()

while not rospy.is_shutdown():

    # ... here I get the data I want to plot into a vector called trans

    marker = Marker()
    marker.header.frame_id = "/neck"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0

    # We add the new marker to the MarkerArray, removing the oldest marker from it when necessary

    markerArray.markers.pop(0)

    markerArray.markers.append(marker)


# Publish the MarkerArray
publisher.publish(markerArray)

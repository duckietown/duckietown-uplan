#!/usr/bin/env python
import rospy
import tf
import math
import random
import geometry_msgs.msg
from colour import Color
red = Color("red")

from std_msgs.msg import ColorRGBA
from uplan_visualization.msg import duckieData
from uplan_visualization.msg import duckieStruct

from visualization_msgs.msg import Marker, MarkerArray

## Subscribe to publish_duckieData, extract each value
# and publish tf frames

duckieMsg = duckieData()
duckieVec = duckieStruct()

thresholdVelocity = 25 #TODO: Move this to duckie.py

"""
# Example
duckieMsg.label = "duckiebot1" #duckie_name
duckieMsg.SE2points = [p1, p2, ...] #trajectory
duckieMsg.value = [v1, v2, ...] #velocity

duckieVec.duckie_data = [duckieMsg]
"""

pub = rospy.Publisher('trajectory_markers', MarkerArray, queue_size=10)

def pub_markers(msg):
    #rospy.loginfo(msg)
    marker_array = MarkerArray()
    duckieVec = duckieStruct()
    duckieVec = msg.duckie_data
    id = 0
    for d in range(len(duckieVec)):
        #rospy.loginfo(duckieVec[d].label)
        marker = Marker()

        marker.header.frame_id = "/duckiebot_link" #TODO: Change
        marker.id = id  #TODO: Change
        marker.ns = "trajectory"
        id = id+1
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.pose.orientation.w = 1.0

        trajVec = duckieVec[d].SE2points
        velocityVec = duckieVec[d].value

        """
        # Color profile:
        v_min = duckie.velocity_profiler.velocity_min
        v_max = duckie.velocity_profiler.velocity_max
        steps = duckie.velocity_profiler.N
        """
        if len(trajVec) != 0:
            clrList = list(red.range_to(Color("blue"),len(velocityVec)))
            vel_sort = list(duckieVec[d].value)
            vel_sort.sort(reverse = True)
            for t in range(1,len(trajVec)):                
                pt = geometry_msgs.msg.Point()
                pt.x = trajVec[t].x
                pt.y = trajVec[t].y
                pt.z = 0
                marker.points.append(pt)
                cl = ColorRGBA()
                cl.a = 1.0 # Don't forget to set the alpha!
                clrList = list(red.range_to(Color("blue"),len(velocityVec)))
                cl.r = clrList[vel_sort.index(velocityVec[t-1])].red
                cl.g = clrList[vel_sort.index(velocityVec[t-1])].green
                cl.b = clrList[vel_sort.index(velocityVec[t-1])].blue
                marker.colors.append(cl)

        marker_array.markers.append(marker)

    pub.publish(marker_array)
    return

def listener():
    rospy.init_node('duckieInfoSub', anonymous=False)
    rospy.Subscriber('duckieData_publisher', duckieStruct, pub_markers)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

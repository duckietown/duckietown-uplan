#!/usr/bin/env python
import rospy
import tf
import math
import random
import geometry_msgs.msg
import numpy as np
from colour import Color
red = Color("red")
blue = Color("blue")

from std_msgs.msg import ColorRGBA
from uplan_visualization.msg import duckieData
from uplan_visualization.msg import duckieStruct

from visualization_msgs.msg import Marker, MarkerArray

## Subscribe to duckieUncertainty_publisher, extract each value
# and publish colored uncertainty markers

duckieMsg = duckieData()
duckieVec = duckieStruct()
# Discretize probabilities to coloring
probList = np.linspace(0,1,11)
clrList = list(blue.range_to(Color("red"),len(probList)))

pub = rospy.Publisher('uncertainty_markers', Marker, queue_size=10)

def pub_markers(msg):
    duckieVec = duckieStruct()
    duckieVec = msg.duckie_data
    marker = Marker()
    pointVec = []
    id = 0
    # For uncertainty, the struct will always be length 1
    for d in range(len(duckieVec)):
        marker.header.frame_id = "/duckiebot_link"
        marker.id = 1
        marker.ns = "uncertainty"
        marker.type = marker.POINTS
        marker.action = marker.ADD

        marker.scale.x = 0.1
        marker.scale.y = 0.1

        marker.pose.orientation.w = 1.0

        # TODO: Color gradient wrt the probability distribution
        ptVec = duckieVec[d].SE2points
        uncertaintyVec = duckieVec[d].value
        if len(ptVec) != 0:
            for t in range(1, len(ptVec)):
                pt = geometry_msgs.msg.Point()
                pt.x = ptVec[t].x
                pt.y = ptVec[t].y
                pt.z = 1.0
                marker.points.append(pt)
                cl = ColorRGBA()
                cl.a = 0.5 # Don't forget to set the alpha!
                indx = int(uncertaintyVec[t-1]*10)
                cl.r = clrList[indx].red
                cl.g = clrList[indx].green
                cl.b = clrList[indx].blue
                marker.colors.append(cl)

    #rospy.loginfo(marker.points)
    pub.publish(marker)
    return

def listener():
    rospy.init_node('duckieUncertaintySub', anonymous=False)
    rospy.Subscriber('duckieUncertainty_publisher', duckieStruct, pub_markers)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

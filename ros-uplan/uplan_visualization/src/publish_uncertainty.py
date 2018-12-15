#!/usr/bin/env python
import rospy
import tf
import math
import random
import geometry_msgs.msg

from std_msgs.msg import String
from std_msgs.msg import Float64
from uplan_visualization.msg import duckieData
from uplan_visualization.msg import duckieStruct

from visualization_msgs.msg import Marker, MarkerArray

## Subscribe to duckieUncertainty_publisher, extract each value
# and publish colored uncertainty markers

duckieMsg = duckieData()
duckieVec = duckieStruct()

pub = rospy.Publisher('uncertainty_markers', Marker, queue_size=10)

def pub_markers(msg):
    duckieVec = duckieStruct()
    duckieVec = msg.duckie_data
    marker = Marker()
    pointVec = []
    id = 0
    # For uncertainty, the struct will always be length 1
    for d in range(len(duckieVec)):
        ptVec = duckieVec[d].SE2points

        marker.header.frame_id = "/duckiebot_link"
        marker.id = 1
        marker.ns = "uncertainty"
        # id = id + 1
        marker.type = marker.POINTS
        marker.action = marker.ADD

        marker.scale.x = 0.1
        marker.scale.y = 0.1

        marker.pose.orientation.w = 1.0

        # TODO: Color gradient wrt the probability distribution
        marker.color.a = 1.0 # Don't forget to set the alpha!
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        for t in range(len(ptVec)):
            pt = geometry_msgs.msg.Point()
            pt.x = ptVec[t].x
            pt.y = ptVec[t].y
            pt.z = 0
            marker.points.append(pt)

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

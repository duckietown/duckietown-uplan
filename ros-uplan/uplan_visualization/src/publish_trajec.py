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

## Subscribe to publish_duckieData, extract each value
# and publish tf frames

duckieMsg = duckieData()
duckieVec = duckieStruct()
traj = geometry_msgs.msg.Pose2D()

"""
# Example
duckieMsg.label = "duckiebot1" #duckie_name
duckieMsg.SE2points = [p1, p2, ...] #trajectory
duckieMsg.value = [v1, v2, ...] #velocity

duckieVec.duckie_data = [duckieMsg]
"""

pub = rospy.Publisher('trajectory_markers', MarkerArray, queue_size=10)

def pub_markers(msg):
    rospy.loginfo(msg)
    duckieVec = duckieStruct()
    marker_array = MarkerArray()
    duckieVec = msg.duckie_data
    id = 0
    for d in range(len(duckieVec)):
        rospy.loginfo(duckieVec[d].label)
        trajVec = duckieVec[d].SE2points

        for t in range(len(trajVec)):
            marker = Marker()

            marker.header.frame_id = "/duckiebot_link" #TODO: Change
            marker.id = id  #TODO: Change
            marker.ns = "trajectory"
            id = id+1
            marker.type = marker.ARROW
            marker.action = marker.ADD

            marker.pose.position.x = trajVec[t].x
            marker.pose.position.y = trajVec[t].y
            marker.pose.position.z = 0

            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            marker.pose.orientation.x = trajVec[t].theta
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 0

            marker.color.a = 1.0 # Don't forget to set the alpha!
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            """
            # # TODO: Convert to Quaternions from Euler
            marker.pose.orientation.x = traj[t][3]
            marker.pose.orientation.y = traj[t][4]
            marker.pose.orientation.z = traj[t][5]
            marker.pose.orientation.w = traj[t][6]
            """
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

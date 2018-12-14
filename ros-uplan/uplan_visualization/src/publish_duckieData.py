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


duckieMsg1 = duckieData()
duckieMsg2 = duckieData()
trac1 = geometry_msgs.msg.Pose2D()
trac2 = geometry_msgs.msg.Pose2D()
duckieVec = duckieStruct()
uncertVec = duckieStruct()

"""
# Example
duckieMsg.duckie_name = "duckiebot1"
duckieMsg.trajectory = [p1, p2, ...]
duckieMsg.velocity = [v1, v2, ...]

duckieVec.duckie_data = [duckieMsg]
"""

## TODO: get angles as quaternions!
trac1.x = 2
trac1.y = 4
trac1.theta = 0.2
trac2.x = 5
trac2.y = 8
trac2.theta = 0.4
duckieMsg1.label = "duckiebot1" #duckie_name
duckieMsg1.SE2points = [trac1, trac2]
duckieMsg1.value = [1.5, 2.5, 1.0] #velocity

duckieMsg2.label = "duckiebot2"
## TODO: get angles as quaternions!
duckieMsg2.SE2points = [trac1, trac2, trac1]
duckieMsg2.value = [1.5, 2.5, 1.0]

duckieVec.duckie_data = [duckieMsg1, duckieMsg2]
uncertVec.duckie_data = [duckieMsg1]

def duckieDataPub():
    pub_traj = rospy.Publisher('duckieData_publisher', duckieStruct, queue_size=10)
    pub_uncert = rospy.Publisher('duckieUncertainty_publisher', duckieStruct, queue_size=10)
    rospy.init_node('duckieInfoPub', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #rospy.loginfo(duckieVec)
        pub_traj.publish(duckieVec)
        pub_uncert.publish(uncertVec)
        rate.sleep()

if __name__ == '__main__':
    try:
        duckieDataPub()
    except rospy.ROSInterruptException:
        pass

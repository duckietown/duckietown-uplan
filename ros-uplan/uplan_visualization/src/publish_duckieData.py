#!/usr/bin/env python
import rospy
import tf
import math
import random
import geometry_msgs.msg
import contracts
contracts.disable_all()

from std_msgs.msg import String
from std_msgs.msg import Float64
from uplan_visualization.msg import duckieData
from uplan_visualization.msg import duckieStruct

import duckietown_world as dw
import duckietown_uplan
from duckietown_uplan.environment.duckie_town import DuckieTown

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
# trac1.x = 2
# trac1.y = 4
# trac1.theta = 0.2
# trac2.x = 5
# trac2.y = 8
# trac2.theta = 0.4
# duckieMsg1.label = "duckiebot1" #duckie_name
# duckieMsg1.SE2points = [trac1, trac2]
# duckieMsg1.value = [1.5, 2.5, 1.0] #velocity
#
# duckieMsg2.label = "duckiebot2"
# ## TODO: get angles as quaternions!
# duckieMsg2.SE2points = [trac1, trac2, trac1]
# duckieMsg2.value = [1.5, 2.5, 1.0]
#
# duckieVec.duckie_data = [duckieMsg1, duckieMsg2]
# uncertVec.duckie_data = [duckieMsg1]

current_map = dw.load_map('4way')
number_of_duckies = 2
duckie_town = DuckieTown(current_map)
duckie_town.augment_graph()
#duckie_town.render_current_graph()
duckie_town.spawn_random_duckie(number_of_duckies)
duckie_town.get_duckie(0).set_visible_path(True)

def execute_simulation():
    duckie_town.create_random_targets_for_all_duckies()
    #duckie_town.step(0.1, display=False)
    duckies_list = []
    duckieVec = duckieStruct()
    duckiePose = []
    duckieLabel = []

    for duckie in duckie_town.get_duckie_citizens():
        duckieMsg1 = duckieData()
        trac1 = geometry_msgs.msg.Pose2D()
        current_SE2 = duckie.get_current_positon()
        trac1.x = current_SE2.p[0]
        trac1.y = current_SE2.p[1]
        trac1.theta = current_SE2.theta
        path = [trac1]
        for c,ctrl_point in enumerate(duckie.get_path()):
            trac1 = geometry_msgs.msg.Pose2D()
            current_SE2 = ctrl_point
            trac1.x = current_SE2.p[0]
            trac1.y = current_SE2.p[1]
            trac1.theta = current_SE2.theta
            if c == 0:
                duckiePose.append((current_SE2.p[0], current_SE2.p[1], current_SE2.theta))
        velocity = [0.5] * (len(path)-1)
        duckieMsg1.label = str(duckie.id)
        duckieLabel.append(duckieMsg1.label)
        duckieMsg1.SE2points = path
        duckieMsg1.value = velocity
        duckies_list.append(duckieMsg1)
    duckieVec.duckie_data = duckies_list
    return duckieVec, duckiePose, duckieLabel

def duckieDataPub():
    pub_traj = rospy.Publisher('duckieData_publisher', duckieStruct, queue_size=10)
    pub_uncert = rospy.Publisher('duckieUncertainty_publisher', duckieStruct, queue_size=10)
    transform_broadcaster = tf.TransformBroadcaster()
    rospy.init_node('duckieInfoPub', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #rospy.loginfo(duckieVec)
        duckieVec, duckiePose, duckieLabel = execute_simulation()
        pub_traj.publish(duckieVec)
        pub_uncert.publish(duckieVec)
        t = rospy.Time.now()
        for i, k in enumerate(duckiePose):
            transform_broadcaster.sendTransform((k[0], k[1], 0), \
            tf.transformations.quaternion_from_euler(0, 0, k[2]), \
            t, duckieLabel[i], "duckiebot_link")
        rate.sleep()

if __name__ == '__main__':
    try:
        duckieDataPub()
    except rospy.ROSInterruptException:
        pass

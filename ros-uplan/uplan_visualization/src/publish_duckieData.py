#!/usr/bin/env python
import rospy
import tf
import math
import random
import geometry_msgs.msg
import contracts
contracts.disable_all()
import yaml

from std_msgs.msg import String
from std_msgs.msg import Float64
from uplan_visualization.msg import duckieData
from uplan_visualization.msg import duckieStruct

import duckietown_world as dw
import duckietown_uplan
from duckietown_uplan.environment.duckie_town import DuckieTown

duckieMsg1 = duckieData()
trac1 = geometry_msgs.msg.Pose2D()
duckieVec = duckieStruct()
uncertVec = duckieStruct()

"""
# Example
duckieMsg.duckie_name = "duckiebot1"
duckieMsg.trajectory = [pose2D, pose2D, ...]
duckieMsg.velocity = [v1, v2, ...]

duckieVec.duckie_data = [duckieMsg]
"""

current_map = dw.load_map('4way')
number_of_duckies = 4
duckie_town = DuckieTown(current_map)
duckie_town.augment_graph()
#duckie_town.render_current_graph()
duckie_town.spawn_random_duckie(number_of_duckies)
duckie_town.reset()
#with open('../src/example.yaml', 'w') as outfile:
#     yaml.dump(a, outfile, default_flow_style=False)
duckie_town.get_duckie(0).set_visible_path(True)

for i in range(1, len(duckie_town.get_duckie_citizens())):
    duckie_town.get_duckie(i).stop_movement()

def execute_simulation():
    duckie_town.create_random_targets_for_all_duckies()
    duckie_town.step(1, display=False)
    duckies_list = []
    duckieVec = duckieStruct()
    duckiePose = []
    duckieLabel = []
    uncertVec = duckieStruct()
    uncert_list = []

    for k, duckie in enumerate(duckie_town.get_duckie_citizens()):
        # POPULATE VELOCITY
        duckieMsg1 = duckieData()
        trac1 = geometry_msgs.msg.Pose2D()
        current_SE2 = duckie.get_current_positon()
        trac1.x = current_SE2.p[0]
        trac1.y = current_SE2.p[1]
        trac1.theta = current_SE2.theta
        path = [trac1]
        for c,ctrl_point_obj in enumerate(duckie.get_path()):
            ctrl_point = ctrl_point_obj[1]['point']
            trac1 = geometry_msgs.msg.Pose2D()
            current_SE2 = ctrl_point
            trac1.x = current_SE2.p[0]
            trac1.y = current_SE2.p[1]
            trac1.theta = current_SE2.theta
            path.append(trac1)
            if c == 0:
                duckiePose.append((current_SE2.p[0], current_SE2.p[1], current_SE2.theta))
        velocity = duckie.current_velocity_profile
        duckieMsg1.label = str(duckie.id)
        duckieLabel.append(duckieMsg1.label)
        if duckie.has_visible_path:
            duckieMsg1.SE2points = path
        else:
            duckieMsg1.SE2points = []
        duckieMsg1.value =  velocity
        duckies_list.append(duckieMsg1)

        # POPUPULATE uncertainty
        duckieUncert1 = duckieData()
        loc = geometry_msgs.msg.Pose2D()
        current_loc = duckie.get_current_positon()
        loc.x = current_loc.p[0]
        loc.y = current_loc.p[1]
        loc.theta = current_loc.theta
        node_loc = [loc]
        uncert_val = []
        uncert_data = duckie.observation_model.get_map_uncertainities(duckie.get_path())
        for node_obj in uncert_data.values(): #(data(point), uncert_value)
            ctrl_point = node_obj[0]['point']
            loc = geometry_msgs.msg.Pose2D()
            current_loc = ctrl_point
            loc.x = current_loc.p[0]
            loc.y = current_loc.p[1]
            loc.theta = current_loc.theta
            node_loc.append(loc)
            uncert_val.append(node_obj[1])
        duckieUncert1.label = str(duckie.id)
        if duckie.has_visible_path:
            duckieUncert1.SE2points = node_loc
            print("uncert val")
            print(uncert_val)
        else:
            duckieUncert1.SE2points = []
        duckieUncert1.value = uncert_val
        uncert_list.append(duckieUncert1)

    uncertVec.duckie_data = uncert_list
    duckieVec.duckie_data = duckies_list
    return duckieVec, duckiePose, duckieLabel, uncertVec

def duckieDataPub():
    pub_traj = rospy.Publisher('duckieData_publisher', duckieStruct, queue_size=10)
    pub_uncert = rospy.Publisher('duckieUncertainty_publisher', duckieStruct, queue_size=10)
    transform_broadcaster = tf.TransformBroadcaster()
    rospy.init_node('duckieInfoPub', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #rospy.loginfo(duckieVec)
        duckieVec, duckiePose, duckieLabel, uncertVec = execute_simulation()
        pub_traj.publish(duckieVec)
        pub_uncert.publish(uncertVec)
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

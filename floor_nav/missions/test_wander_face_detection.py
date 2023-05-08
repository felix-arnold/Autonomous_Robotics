#!/usr/bin/env python3
# ROS specific imports
import roslib; roslib.load_manifest('floor_nav')
import rospy
from math import *
from task_manager_lib.TaskClient import *

rospy.init_node('task_client')
server_node = rospy.get_param("~server","/task_server")
default_period = rospy.get_param("~period",0.05)
tc = TaskClient(server_node,default_period)
rospy.loginfo("Mission connected to server: " + server_node)

scale=2.0
vel=0.5

tc.WaitForAuto()

while True:

    w4face = tc.WaitForFace(foreground=False)
    tc.addCondition(ConditionIsCompleted("Face detector", tc, w4face))
    
    try:
        tc.Wander()

    except TaskConditionException as e:
        rospy.loginfo("Path following interrupted on condition: %s" % \
                " or ".join([str(c) for c in e.conditions]))
        tc.StareAtFace()
        tc.Wait(duration = 3)
        tc.SetHeading(target = -30, relative = True, max_angular_velocity = 5)
        tc.clearConditions()

rospy.loginfo("Mission completed")

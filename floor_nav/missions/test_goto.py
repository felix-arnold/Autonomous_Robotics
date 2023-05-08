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

scale=4
theta=pi/2
vel=0.5

tc.WaitForAuto()
try:
    
    tc.GoToPose(goal_x=scale, goal_y=0, goal_theta=theta,relative=True, max_velocity=vel, smart = True,  holonomic = True)
    tc.Wait(duration=1.0)
    tc.GoToPose(goal_x=0, goal_y=scale, goal_theta=-theta, relative=True, max_velocity=vel, smart = True,  holonomic = True)
    tc.Wait(duration=1.0)
    tc.GoToPose(goal_x=-scale, goal_y=0, goal_theta=theta, relative=True, max_velocity=vel, smart = True,  holonomic = True)
    tc.Wait(duration=1.0)
    tc.GoToPose(goal_x=0, goal_y=-scale, goal_theta=-theta, relative=True, max_velocity=vel, smart = True,  holonomic = True)

    tc.GoTo(goal_x=scale, goal_y=0,relative=True, max_velocity=vel, holonomic = True)
    tc.Wait(duration=1.0)
    tc.GoTo(goal_x=0, goal_y=scale, relative=True, max_velocity=vel, holonomic = True)
    tc.Wait(duration=1.0)
    tc.GoTo(goal_x=-scale, goal_y=0, relative=True, max_velocity=vel, holonomic = True)
    tc.Wait(duration=1.0)
    tc.GoTo(goal_x=0, goal_y=-scale, relative=True, max_velocity=vel, holonomic = True)
    
    tc.GoToPose(goal_x=scale, goal_y=0, goal_theta=theta,relative=True, max_velocity=vel, smart = False,  holonomic = True)
    tc.Wait(duration=1.0)
    tc.GoToPose(goal_x=0, goal_y=scale, goal_theta=-theta, relative=True, max_velocity=vel, smart = False,  holonomic = True)
    tc.Wait(duration=1.0)
    tc.GoToPose(goal_x=-scale, goal_y=0, goal_theta=theta, relative=True, max_velocity=vel, smart = False,  holonomic = True)
    tc.Wait(duration=1.0)
    tc.GoToPose(goal_x=0, goal_y=-scale, goal_theta=-theta, relative=True, max_velocity=vel, smart = False,  holonomic = True)

    

except TaskException as e:
    rospy.logerr("Exception caught: " + str(e))

if not rospy.core.is_shutdown():
    tc.SetManual()


rospy.loginfo("Mission completed")

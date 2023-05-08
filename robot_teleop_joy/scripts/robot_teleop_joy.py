#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def joyGetInputs(joy):

    #for switch joycon
    #crossAxesIndex = 4
    #for logitech joycon
    crossAxesIndex = 6

    linear = (joy.axes[crossAxesIndex+1] if joy.axes[crossAxesIndex+1] else joy.axes[1]) * (joy.buttons[1]*2+1)

    #for turtlesim
    #angular = (joy.axes[crossAxesIndex] if joy.axes[crossAxesIndex] else joy.axes[0]) * (joy.buttons[1]*2+1)
    #for coppelia robot
    angular = (-joy.axes[crossAxesIndex] if -joy.axes[crossAxesIndex] else -joy.axes[0]) * (joy.buttons[1]*2+1)

    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    robPub.publish(twist)

    print("linear: ", linear,"   angular: ", angular)

def teleopRobot():
    global robPub
    #for turtlesim
    #robPub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size = 10)
    #for coppelia robot
    robPub = rospy.Publisher('vrep/twistCommand', Twist, queue_size = 10)

    joySub = rospy.Subscriber("joy", Joy, joyGetInputs)
    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node("robotTeleop")
        teleopRobot()
    except rospy.ROSInterruptException:
        pass

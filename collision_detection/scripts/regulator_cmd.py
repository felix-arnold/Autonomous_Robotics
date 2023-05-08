#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


j=Twist()

def send(joy_msg):
  pub=rospy.Publisher('teleop_velocity_smoother/raw_cmd_vel',Twist,queue_size=1)
  pub.publish(joy_msg)


def callback_laser(laser_msg):
  global j
  d=10
  l=int(len(laser_msg.ranges)/4) # on garde les valeurs du milieu pour ne pas regarder trop grand
  for i in range (l,3*l+1):
    if laser_msg.ranges[i]<d:
      d=laser_msg.ranges[i]
    if d>2:
      send(j)
    elif d<0.6:
      j.linear.x=0
      send(j)
    else:
      j.linear.x=math.sin(d)*j.linear.x
      send(j)

def callback_cmd(joy_msg):
    global j
    j=joy_msg

	
def receive():
	rospy.Subscriber('cmd_vel',Twist, callback_cmd)
	rospy.Subscriber('scan',LaserScan, callback_laser)


if __name__=='__main__':
    rospy.init_node('cmd_regulator_node')
    receive()
    rospy.spin()
    
    
    


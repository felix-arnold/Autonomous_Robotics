import roslib; roslib.load_manifest('ar_mapping_base')
import rospy
from numpy import *
from numpy.linalg import inv
from math import pi, sin, cos
from visualization_msgs.msg import Marker, MarkerArray
import tf
import threading

import rover_driver_base
from rover_driver_base.rover_kinematics import *

# 
# Y = G(X)
# Cov(X) = C
# 
# Cov(Y) = dG/dX C dG/dX^T



class Landmark:
    def __init__(self, Z, X , R):
        # Initialise a landmark based on measurement Z, 
        # current position X and uncertainty R
        # TODO
        # L = g(X,Z)
        # Cov(L) = dG/dX Cov(X) dG/dX^T + dG/dZ Cov(Z) dG/dZ^T
        
        rot=array([[cos(X[2]), -sin(X[2])], [sin(X[2]), cos(X[2])]])
        self.L =mat([X[0],X[1]]+dot(rot,Z)) 
        self.P =mat(dot(dot(rot,R),transpose(rot)))

    def update(self,Z, X, R):
        # Update the landmark based on measurement Z, 
        # current position X and uncertainty R
        # TODO

        H=array([[cos(X[2]), sin(X[2])], [-sin(X[2]), cos(X[2])]])
        tH=transpose(H)
        K=dot(dot(self.P,tH),inv(dot(dot(H,self.P),tH)+R))
        I=identity(2)
        X2=array([X[0],X[1]])
        h=dot(H,(self.L-X2))
        self.P=dot((I-dot(K,H)),self.P)
        self.L=self.L+dot(K,(Z-h))
        return self.L

class MappingKF:
    def __init__(self):
        self.lock = threading.Lock()
        self.marker_list = {}
        self.marker_pub = rospy.Publisher("~landmarks",MarkerArray,queue_size=1)

    def update_ar(self, Z, X, Id, uncertainty):
        self.lock.acquire()
        print("Update: Z="+str(Z.T)+" X="+str(X.T)+" Id="+str(Id))
        R = mat(diag([uncertainty,uncertainty]))
        # Take care of the landmark Id observed as Z from X
        # self.marker_list is expected to be a dictionary of Landmark
        # such that current landmark can be retrieved as self.marker_list[Id] 
        # At initialisation, self.marker_list is empty
        # TODO

        if Id in self.marker_list:
            self.marker_list[Id].update(Z, X, R)
        else:
            self.marker_list[Id] = Landmark(Z, X, R)

        self.lock.release()


    def publish(self, target_frame, timestamp):
        ma = MarkerArray()
        for id in self.marker_list:
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = target_frame
            marker.ns = "landmark_kf"
            marker.id = id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            Lkf = self.marker_list[id]
            marker.pose.position.x = Lkf.L[0,0]
            marker.pose.position.y = Lkf.L[1,0]
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 1
            marker.pose.orientation.w = 0
            marker.scale.x = max(3*sqrt(Lkf.P[0,0]),0.05)
            marker.scale.y = max(3*sqrt(Lkf.P[1,1]),0.05)
            marker.scale.z = 0.5;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.lifetime.secs=3;
            ma.markers.append(marker)
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = target_frame
            marker.ns = "landmark_kf"
            marker.id = 1000+id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            Lkf = self.marker_list[id]
            marker.pose.position.x = Lkf.L[0,0]
            marker.pose.position.y = Lkf.L[1,0]
            marker.pose.position.z = 1.0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 1
            marker.pose.orientation.w = 0
            marker.text = str(id)
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 0.2
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.lifetime.secs=3;
            ma.markers.append(marker)
        self.marker_pub.publish(ma)


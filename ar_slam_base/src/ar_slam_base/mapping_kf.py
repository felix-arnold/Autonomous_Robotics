import roslib; roslib.load_manifest('ar_mapping_base')
import rospy
from numpy import *
from numpy import mod
from numpy.linalg import inv
from math import pi, sin, cos
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, PoseStamped
import tf
import threading

import ar_loc_base
from ar_loc_base.rover_kinematics import *



class MappingKF(RoverKinematics):
    def __init__(self, initial_pose, initial_uncertainty):
        RoverKinematics.__init__(self)
        self.lock = threading.Lock()
        self.X = mat(vstack(initial_pose))
        self.P = mat(diag(initial_uncertainty))
        self.idx = {}
        self.pose_pub = rospy.Publisher("~pose",PoseStamped,queue_size=1)
        self.marker_pub = rospy.Publisher("~landmarks",MarkerArray,queue_size=1)

    def getRotation(self, theta):
        R = mat(zeros((2,2)))
        R[0,0] = cos(theta); R[0,1] = -sin(theta)
        R[1,0] = sin(theta); R[1,1] = cos(theta)
        return R

    
    def predict(self, motor_state, drive_cfg, encoder_precision):

        self.lock.acquire()
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            self.lock.release()
            return (self.X, self.P)
        # print("-"*32)
        # then compute odometry using least square
        iW = self.prepare_inversion_matrix(drive_cfg)
        S = self.prepare_displacement_matrix(self.motor_state,motor_state,drive_cfg)
        self.motor_state.copy(motor_state)
        
        # Implement Kalman prediction here
        theta = self.X[2,0]
        Rtheta = mat([[cos(theta), -sin(theta), 0], 
                      [sin(theta),  cos(theta), 0],
                      [         0,           0, 1]])
        DeltaX = iW*S
        # Update the state using odometry (same code as for the localisation
        # homework), but we only need to deal with a subset of the state:
        # TODO
        # self.X[0:3,0] = ...
        # self.P[0:3,0:3] = ...

        A=array([[1, 0, -sin(self.X[2])*DeltaX[1,0]-cos(self.X[2])*DeltaX[0,0]], 
                 [0, 1,  cos(self.X[2])*DeltaX[0,0]-sin(self.X[2])*DeltaX[1,0]], 
                 [0, 0,                                                      1]])
        tA=transpose(A)
        B=dot(Rtheta,iW)
        tB=transpose(B)

        Qu= eye(12)*(encoder_precision**2)
        Q = eye(3)*1e-6

        self.X[0:3,0]=self.X[0:3,0]+dot(dot(Rtheta,iW),S)
        self.P[0:3,0:3]=mat(dot(dot(A,self.P[0:3,0:3]),tA)+dot(dot(B,Qu),tB)+Q)

        self.lock.release()
        return (self.X,self.P)


    def update_ar(self, Z, id, uncertainty):
        # Landmark id has been observed as Z with a given uncertainty
        self.lock.acquire()
        print("Update: Z="+str(Z.T)+" X="+str(self.X.T)+" Id="+str(id))
        # Update the full state self.X and self.P based on landmark id
        # be careful that this might be the first time that id is observed
        # TODO
        # self.X = ...
        # self.P = ...

        print("Id:", id)

        rot=array([[cos(self.X[2]), -sin(self.X[2])], [sin(self.X[2]), cos(self.X[2])]])
        R = mat(diag([uncertainty,uncertainty]))
        if id in self.idx:

            L = self.X[self.idx[id]:self.idx[id]+2] 
            dL = L - self.X[0:2]
            # print("dL ", dL)
            H2=array([[-cos(self.X[2]), -sin(self.X[2]), -dL[0,0]*sin(self.X[2])+dL[1,0]*cos(self.X[2])], 
                      [sin(self.X[2]),  -cos(self.X[2]), -dL[0,0]*cos(self.X[2])-dL[1,0]*sin(self.X[2])]])
            H1=array([[cos(self.X[2]), sin(self.X[2])], [-sin(self.X[2]), cos(self.X[2])]])
            H = concatenate((H2, zeros((2, self.idx[id]-3)), H1, zeros((2, len(self.X)-self.idx[id]-2))), axis = 1)
            # print("H ", H)
            
            tH=transpose(H)
            K=dot(dot(self.P,tH),inv(dot(dot(H,self.P),tH)+R))
            # print("K ", K)
            X2=array([self.X[0,0],self.X[1,0]])
            # print("X2 ", X2)
            h=dot(H1,(L-X2))
            # print("h ", h)
            epsilon = 1e-10
            self.P=dot((identity(len(self.X))-dot(K,H)),self.P)+epsilon*identity(len(self.P))
            # print("P ", self.P)
            self.X = self.X+dot(K,(Z-dot(transpose(rot), L-self.X[0:2])))
            # print("X ", self.X) 

        else:
            print("Id:", id)
            self.idx[id] = len(self.X)
            print(self.X, self.X[0:2,0]+dot(rot,Z))
            self.X=concatenate((self.X, self.X[0:2,0]+dot(rot,Z)), axis=0) 
            # print("X ", self.X)           
            Ptmp = mat(dot(dot(rot,R),transpose(rot)))
            # print("Ptmp ", Ptmp)
            self.P = concatenate((concatenate((self.P, zeros((len(self.X)-2, 2))), axis=1), concatenate((zeros((2, len(self.X)-2)), Ptmp), axis=1)), axis=0)
            # print("P:\n", self.P)

        self.lock.release()
        return (self.X,self.P)

    def update_compass(self, Z, uncertainty):
        self.lock.acquire()
        print("Update: S="+str(Z)+" X="+str(self.X.T))
        # Update the full state self.X and self.P based on compass measurement
        # TODO
        # self.X = ...
        # self.P = ...

        H=mat([0,0,1])
        P = self.P[0:3,0:3]
        X = self.X[0:3,0]

        K = dot(dot(P,transpose(H)),inv(dot(dot(H,P),transpose(H))+uncertainty))
        X = X + dot(K, mod(Z-dot(H,X) + pi, 2*pi)-pi)
        P = dot((identity(3)-dot(K,H)), P)
        print("P:\n", P)
        self.X[0:3, 0] = X
        self.P[0:3,0:3] = P

        self.lock.release()
        return (self.X,self.P)


    def publish(self, target_frame, timestamp):
        pose = PoseStamped()
        pose.header.frame_id = target_frame
        pose.header.stamp = timestamp
        pose.pose.position.x = self.X[0,0]
        pose.pose.position.y = self.X[1,0]
        pose.pose.position.z = 0.0
        Q = tf.transformations.quaternion_from_euler(0, 0, self.X[2,0])
        pose.pose.orientation.x = Q[0]
        pose.pose.orientation.y = Q[1]
        pose.pose.orientation.z = Q[2]
        pose.pose.orientation.w = Q[3]
        self.pose_pub.publish(pose)
        ma = MarkerArray()
        marker = Marker()
        marker.header = pose.header
        marker.ns = "kf_uncertainty"
        marker.id = 5000
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.pose.position.z = -0.1
        marker.scale.x = 3*sqrt(self.P[0,0])
        marker.scale.y = 3*sqrt(self.P[1,1]);
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        ma.markers.append(marker)
        for id in self.idx:
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = target_frame
            marker.ns = "landmark_kf"
            marker.id = id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            l = self.idx[id]
            marker.pose.position.x = self.X[l,0]
            marker.pose.position.y = self.X[l+1,0]
            marker.pose.position.z = -0.1
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 1
            marker.pose.orientation.w = 0
            marker.scale.x = 3*sqrt(self.P[l,l])
            marker.scale.y = 3*sqrt(self.P[l+1,l+1]);
            marker.scale.z = 0.1;
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
            marker.pose.position.x = self.X[l+0,0]
            marker.pose.position.y = self.X[l+1,0]
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


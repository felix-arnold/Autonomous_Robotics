
#include <vector>
#include <string>
#include <map>
#include <list>


#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <occgrid_planner_base/Trajectory.h>

class PathOptimizer {
    protected:
        ros::NodeHandle nh_;
        ros::Subscriber path_sub_;
        ros::Publisher traj_pub_;
        double velocity_;
        double max_acceleration_;
        double max_braking_;

        void path_cb(const nav_msgs::PathConstPtr & msg) {
            std::vector<float> omega(msg->poses.size(),0.0);
            std::vector<float> s(msg->poses.size(),0.0);
            std::vector<float> heading(msg->poses.size(),0.0);
            heading[0] = tf::getYaw(msg->poses[0].pose.orientation);
            for (unsigned i=1;i<msg->poses.size();i++) {
                const geometry_msgs::Point& P0 = msg->poses[i-1].pose.position;
                const geometry_msgs::Point& P1 = msg->poses[i].pose.position;
                double ds = hypot(P1.y-P0.y,P1.x-P0.x);
                double dt = ds / velocity_;
                heading[i] = tf::getYaw(msg->poses[i].pose.orientation);
                if (dt < 0.01){
                    if ((heading[i]-heading[i-1]) < -1 ) { 
                        omega[i] = -1.0;
                    } else if ((heading[i]-heading[i-1]) > 1 ) { 
                        omega[i] = 1.0;
                    } else if ((heading[i]-heading[i-1]) < 0) {
                        omega[i] = 1.0;
                    } else {
                        omega[i] = -1.0;
                    }     
                } else {
                    omega[i] = remainder(heading[i]-heading[i-1], 2*M_PI) / dt;
                }
                s[i] = s[i-1] + ds;
            }

            occgrid_planner_base::Trajectory output;
            output.header = msg->header;
            output.Ts.resize(msg->poses.size());
            output.Ts[0].header.stamp = output.header.stamp;
            for (unsigned i=0;i<msg->poses.size();i++) {
                output.Ts[i].header = msg->poses[i].header;
                if (i>0) {
                    if((abs(s[i]-s[i-1]) < 0.01)) {
                        output.Ts[i].header.stamp = output.Ts[i-1].header.stamp + ros::Duration((abs(remainder(heading[i]-heading[i-1], 2*M_PI))/abs(omega[i])));
                    } else {       
                        output.Ts[i].header.stamp = output.Ts[i-1].header.stamp + ros::Duration((s[i]-s[i-1])/velocity_);
                    }
                }
                output.Ts[i].pose.position = msg->poses[i].pose.position;
                tf::Quaternion q = tf::createQuaternionFromRPY(0,0,heading[i]);
                tf::quaternionTFToMsg(q, output.Ts[i].pose.orientation);
                if ((i <msg->poses.size()-1) && (abs(s[i+1]-s[i]) < 0.01)){
                    output.Ts[i].twist.linear.x = 0;
                    output.Ts[i].twist.angular.z = omega[i+1];
                }else{
                    output.Ts[i].twist.linear.x = velocity_;
                    output.Ts[i].twist.angular.z = 0;
                    ROS_INFO("NO %i", i);
                }    
            }
            if (output.Ts.size()>0) {
                unsigned int j = output.Ts.size() - 1;
                output.Ts[j].twist.linear.x = 0;
                output.Ts[j].twist.angular.z = 0;
            }
            traj_pub_.publish(output);
            ROS_INFO("Optimized path into a trajectory");
        }

    public:
        PathOptimizer() : nh_("~") {
            nh_.param("velocity",velocity_,1.0);
            nh_.param("max_acceleration",max_acceleration_,1.0);
            nh_.param("max_braking",max_braking_,1.0);
            path_sub_ = nh_.subscribe<nav_msgs::Path>("path",1,&PathOptimizer::path_cb,this);
            traj_pub_ = nh_.advertise<occgrid_planner_base::Trajectory>("trajectory",1,true);
        }
};

int main(int argc, char * argv[]) {
    ros::init(argc,argv,"path_optimizer");
    PathOptimizer po;
    ros::spin();
}




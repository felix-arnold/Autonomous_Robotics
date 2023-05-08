#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <iostream>

class TeleopRobot {

public:
    TeleopRobot();

private :
    void joyGetInputs(const sensor_msgs::Joy::ConstPtr& joy);
    ros::Publisher robPub;
    ros::Subscriber joySub;
};

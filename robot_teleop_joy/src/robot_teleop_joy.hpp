#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <iostream>

class TeleopRobot {

public:
    TeleopRobot();

private :
    void joyGetInputs(const sensor_msgs::Joy::ConstPtr& joy);
    void publish();
    ros::NodeHandle nh;
    geometry_msgs::Twist twist;
    ros::Publisher robPub;
    ros::Subscriber joySub;
    std::string publishTopicName;
    int stickIndexLinear, stickIndexAngular, dpadIndexLinear, dpadIndexAngular, buttonIndex;
    bool angularMode;
    double buttonVelocity;
    float linear, angular;
};

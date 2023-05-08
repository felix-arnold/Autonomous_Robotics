#include "robot_teleop_joy.hpp"

TeleopRobot::TeleopRobot() {
    ros::NodeHandle nh;

    //for turtlesim
    //robPub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    //for coppelia robot
    robPub = nh.advertise<geometry_msgs::Twist>("vrep/twistCommand", 1);

    joySub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopRobot::joyGetInputs, this);
    ros::spin();
}

void TeleopRobot::joyGetInputs(const sensor_msgs::Joy::ConstPtr& joy) {

    //for switch joystick
    //int crossAxesIndex = 4;
    //for logitech joystick
    int crossAxesIndex = 6;

    float linear = (joy->axes[crossAxesIndex+1] ? joy->axes[crossAxesIndex+1] : joy->axes[1]) * (joy->buttons[1]*2+1);

    //for turtlesim
    //float angular = (joy->axes[crossAxesIndex] ? joy->axes[crossAxesIndex] : joy->axes[0]) * (joy->buttons[1]*2+1);
    //for coppelia robot
    float angular = (-joy->axes[crossAxesIndex] ? -joy->axes[crossAxesIndex] : -joy->axes[0]) * (joy->buttons[1]*2+1);

    geometry_msgs::Twist twist;
    twist.linear.x = linear;
    twist.angular.z = angular;
    robPub.publish(twist);

    std::cout << "linear: " << linear << "   angular: " << angular << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "robotTeleop");
    TeleopRobot teleopRobot;

    return 0;
}
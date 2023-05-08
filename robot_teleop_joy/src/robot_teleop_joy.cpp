#include "robot_teleop_joy.hpp"

TeleopRobot::TeleopRobot() {    
    nh.param("publishTopicName", publishTopicName, (std::string) "vrep/twistCommand");
    nh.param("stickIndexLinear", stickIndexLinear, 1);
    nh.param("stickIndexAngular", stickIndexAngular, 0);
    nh.param("dpadIndexLinear", dpadIndexLinear, 5);
    nh.param("dpadIndexAngular", dpadIndexAngular, 4);
    nh.param("inverseAngularMode", angularMode, false);
    nh.param("buttonIndex", buttonIndex, 1);
    nh.param("buttonVelocity", buttonVelocity, 0.5);

    robPub = nh.advertise<geometry_msgs::Twist>(publishTopicName, 1);
    joySub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopRobot::joyGetInputs, this);

    ros::Timer timer = nh.createTimer(ros::Duration(0.1), boost::bind(&TeleopRobot::publish, this));

    ros::spin();
}

void TeleopRobot::joyGetInputs(const sensor_msgs::Joy::ConstPtr& joy) {

    linear = (joy->axes[dpadIndexLinear] ? joy->axes[dpadIndexLinear] : joy->axes[stickIndexLinear]) * (joy->buttons[buttonIndex]*(buttonVelocity-1)+1);
    angular = (joy->axes[dpadIndexAngular] ? (1-2*(int)angularMode)*joy->axes[dpadIndexAngular] : (1-2*(int)angularMode)*joy->axes[stickIndexAngular]) * (joy->buttons[buttonIndex]*(buttonVelocity-1)+1);

    twist.linear.x = linear;
    twist.angular.z = angular;
}

void TeleopRobot::publish() {
    robPub.publish(twist);
    std::cout << "  Linear value: " << linear << "\n  Angular value: " << angular << "\n----------" << std::endl;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "robotTeleop");
    TeleopRobot teleopRobot;

    return 0;
}
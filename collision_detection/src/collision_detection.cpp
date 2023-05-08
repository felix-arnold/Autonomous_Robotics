#include "collision_detection.hpp"

CollisionDetector::CollisionDetector() {
    senSub = nh.subscribe<sensor_msgs::Twist>(senTopic, 10, &CollisionDetector::detectCollision, this);
    joySub = nh.subscribe<sensor_msgs::Joy>(telTopic, 10, &TeleopRobot::controlRobot, this);
    robPub = nh.advertise<geometry_msgs::Twist>(robPub, 1);
}

void CollisionDetector::detectCollision(const sensor_msgs::PointCloud2ConstPtr msg) {
    pcl::fromROSMsg(*msg, pc);
    for (auto i : pc) {
        printf(“x %f y %f z %f\n”,i.x,i.y,i.z);
        /*if (i[y] < 0.75)
            speedRatio = (i[y]-0.5)*4;
        else
            speedRatio = 1;*/
    }
}

void CollisionDetector::controlRobot(const sensor_msgs::TwistPtr msg) {
    twist = msg;
    twist.linear.x = twist.linear.x * speedRatio;
    robPub.publish(twist)
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "CollisionDetector");
    CollisionDetector collisionDetector;

    return 0;
}
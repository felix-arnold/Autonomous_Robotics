#include "iostream"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

class CollisionDetector {

public:
    CollisionDetector();

private :
    void detectCollision(const sensor_msgs::PointCloud2ConstPtr msg);
    void controlRobot(const sensor_msgs::TwistPtr msg);

    ros::NodeHandle nh;
    pcl::PointCloud <pcl::PointXYZ> pc;
    ros::Publisher robPub;
    ros::Subscriber senSub, telSub;
    geometry_msgs::Twist twist;
    std::string robTopic, senTopic, telTopic;
    unsigned double speedRatio = 1;
};

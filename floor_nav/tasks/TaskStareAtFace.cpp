#include <math.h>
#include "TaskStareAtFace.h"
#include "floor_nav/TaskStareAtFaceConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GOTO

TaskIndicator TaskStareAtFace::initialise() 
{
    ROS_INFO("Setting heading to %.2f deg", cfg.target*180./M_PI);
    if ((env->getFaceInfo().faces).empty())
        terminate();
    initial_heading = 0.0;
    int i = 0;
    for (auto f : (env->getFaceInfo()).faces) {
        if (f.width > env->getFaceInfo().faces[faceIndex].width)
            faceIndex = i;
        i ++;
    }
    return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskStareAtFace::iterate()
{
    const geometry_msgs::Pose2D & tpose = env->getPose2D();
    sensor_msgs::RegionOfInterest face = env->getFaceInfo().faces[faceIndex];
    float alpha = 64 - ((float)face.x_offset + ((float)face.width)/2);
    if (fabs(alpha) < cfg.angle_threshold) {
		return TaskStatus::TASK_COMPLETED;
    }
    float rot = cfg.k_theta*(alpha/300);
    if (rot > cfg.max_angular_velocity) rot = cfg.max_angular_velocity;
    if (rot <-cfg.max_angular_velocity) rot =-cfg.max_angular_velocity;
    env->publishVelocity(0.0, rot);
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskStareAtFace::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryStareAtFace);

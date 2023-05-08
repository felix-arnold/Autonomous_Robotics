#include <math.h>
#include "TaskGoToPose.h"
#include "floor_nav/TaskGoToPoseConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GOTO
#ifdef DEBUG_GOTO
#warning Debugging task GOTO
#endif


TaskIndicator TaskGoToPose::initialise() 
{
    ROS_INFO("Going to %.2f %.2f with angle %.2f",cfg.goal_x,cfg.goal_y,cfg.goal_theta);
    if (cfg.relative) {
        const geometry_msgs::Pose2D & tpose = env->getPose2D();
        x_init = tpose.x;
        y_init = tpose.y;
        theta_init = tpose.theta;
    } else {
        x_init = 0.0;
        y_init = 0.0;
        theta_init = 0.0;
    }
    return TaskStatus::TASK_INITIALISED;
}


TaskIndicator TaskGoToPose::iterate()
{
    const geometry_msgs::Pose2D & tpose = env->getPose2D();
    double r = hypot(y_init + cfg.goal_y-tpose.y,x_init + cfg.goal_x-tpose.x);
    double angle = cfg.goal_theta-tpose.theta + theta_init;
    double alpha = remainder(atan2((y_init + cfg.goal_y-tpose.y),x_init + cfg.goal_x-tpose.x)-tpose.theta,2*M_PI);
    double beta=-tpose.theta-alpha;
    if (!cfg.smart) {
        beta=0;
        if (r < cfg.dist_threshold) {
            if (fabs(angle) < cfg.angle_threshold){
                return TaskStatus::TASK_COMPLETED;
            }
            float rot = cfg.k_alpha*angle;
            if (rot > cfg.max_angular_velocity) rot = cfg.max_angular_velocity;
            if (rot <-cfg.max_angular_velocity) rot =-cfg.max_angular_velocity;
            if (cfg.holonomic){
                env->publishVelocity(0.0,0.0,rot);
            }
            else{
                env->publishVelocity(0.0,rot);
            }
	        return TaskStatus::TASK_RUNNING;
        }
    }
    else {
        if (r < cfg.dist_threshold && fabs(angle) < cfg.angle_threshold)
            return TaskStatus::TASK_COMPLETED;
    }

    if (cfg.holonomic) {
            double dx = x_init+cfg.goal_x-tpose.x;
            double dy = y_init+cfg.goal_y-tpose.y;
            double dtheta = cfg.k_theta * (theta_init+cfg.goal_theta-tpose.theta);
            double vel_x = cfg.k_v * (cos(tpose.theta) * dx + sin(tpose.theta) * dy);
            double vel_y = cfg.k_v * (-sin(tpose.theta) * dx + cos(tpose.theta) * dy);
            double v = hypot(vel_x, vel_y);
            if (abs(vel_x)  > cfg.max_velocity) vel_x = cfg.max_velocity/v*vel_x;
            if (abs(vel_y) > cfg.max_velocity) vel_y = cfg.max_velocity/v*vel_y;
            env->publishVelocity(vel_x,vel_y, dtheta);
    }
    else {
        if (fabs(alpha) > M_PI/9) {
            double rot = ((alpha>0)?+1:-1)*cfg.max_angular_velocity;
            env->publishVelocity(0,rot);
        }
        else {
            double vel = cfg.k_v * r;
            double rot = std::max(std::min(cfg.k_alpha*alpha+cfg.k_beta*beta,cfg.max_angular_velocity),-cfg.max_angular_velocity);
            if (vel > cfg.max_velocity)
            vel = cfg.max_velocity;
            env->publishVelocity(vel, rot);
        }
    }
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskGoToPose::terminate()
{
    if (cfg.holonomic){
        env->publishVelocity(0,0,0);
    }
    else{
        env->publishVelocity(0,0);
    }
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryGoToPose);

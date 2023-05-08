#include <math.h>
#include "TaskWaitForFace.h"
#include "floor_nav/TaskWaitForFaceConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;


TaskIndicator TaskWaitForFace::iterate()
{
    const face& detectedFaces = env->getFaceInfo();
    const geometry_msgs::Pose2D & tpose = env->getPose2D();
    if (!detectedFaces.faces.empty()) {
        ROS_INFO("Detected Face at %.2f %.2f",tpose.x, tpose.y);
		return TaskStatus::TASK_COMPLETED;
    }
	return TaskStatus::TASK_RUNNING;
}

DYNAMIC_TASK(TaskFactoryWaitForFace);

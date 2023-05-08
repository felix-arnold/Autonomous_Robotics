#ifndef TASK_MOVE_GOAL_H
#define TASK_MOVE_GOAL_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "task_manager_action/TaskActionMoveGoal.h"

using namespace task_manager_lib;
using namespace task_manager_action;

// There is no move_goal for turtlesim, this is to test the principle of the
// generic TaskActionMoveGoal
namespace floor_nav {

    struct TurtleSimGetGoalPublisher {
        TurtleSimGetGoalPublisher() {}
        ros::Publisher operator()(task_manager_lib::TaskEnvironmentPtr env, const std::string & topic_name) {
#if 1
            ros::NodeHandle & nh = env->getNodeHandle();
            return nh.advertise<geometry_msgs::PoseStamped>(topic_name,1);
#else
            SimTasksEnvPtr tenv = boost::dynamic_pointer_cast<SimTasksEnv>(env);
            return tenv->getGoalPublisher();
#endif
        }
    };


#if 0
    class TaskMoveGoal : public TaskActionMoveGoal<SimTasksEnv,TurtleSimGetGoalPublisher>
    {
            TaskMoveGoal(TaskDefinitionPtr def, TaskEnvironmentPtr env) :
                TaskActionMoveGoal<SimTasksEnv,TurtleSimGetGoalPublisher>(def,env) {}
            virtual ~TaskMoveGoal() {};
    };
#else
    class TaskMoveGoal : public TaskActionMoveGoal<SimTasksEnv>
    {
            TaskMoveGoal(TaskDefinitionPtr def, TaskEnvironmentPtr env) :
                TaskActionMoveGoal<SimTasksEnv>(def,env) {}
            virtual ~TaskMoveGoal() {};
    };
#endif

    class TaskFactoryMoveGoal : public TaskFactoryActionMoveGoal<SimTasksEnv>
    {

        public:
            TaskFactoryMoveGoal(TaskEnvironmentPtr env) :
                TaskFactoryActionMoveGoal<SimTasksEnv>(env) {}
            virtual ~TaskFactoryMoveGoal() {};
    };
};

#endif // TASK_MOVE_GOAL_H

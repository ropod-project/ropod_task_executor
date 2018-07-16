#ifndef TASK_EXECUTOR_H
#define TASK_EXECUTOR_H

#include <ros/ros.h>
#include <ropod_ros_msgs/Task.h>
#include <ropod_ros_msgs/ropod_demo_plan.h>

/**
 * Executor of tasks sent by fleet management.
 * Also publishes feedback when intermediate actions are complete,
 * and when the task is complete.
 *
 * @author Santosh Thoduka
 */
class TaskExecutor
{
private:
    ros::NodeHandle nh;

    /**
     * Subscriber for task message
     */
    ros::Subscriber task_sub;

    /**
     * Publisher for task and action feedback
     */
    ros::Publisher task_feedback_pub;

    /**
     * Publisher for sending go_to actions to navigation
     */
    ros::Publisher action_goto_pub;

    /**
     * Publisher for sending dock actions
     */
    ros::Publisher action_dock_pub;

    /**
     * Publisher for sending undock actions
     */
    ros::Publisher action_undock_pub;

    /**
     * Currently executing task
     */
    ropod_ros_msgs::Task::Ptr current_task;

    /**
     * Flag indicating if a task has been received
     */
    bool received_task;

    /**
     * Flag indicating whether task execution is ongoing
     */
    bool execution_ongoing;

    /**
     * Flag indicating whether an action within the task is being
     * executing currently
     */
    bool action_ongoing;

    /**
     * Index of current action being executed
     */
    int current_action_index;

public:
    TaskExecutor();
    virtual ~TaskExecutor();

    /**
     * Subscriber callback for task messages
     */
    void taskCallback(const ropod_ros_msgs::Task::Ptr &msg);

    /**
     * Main loop; dispatches actions one by one
     */
    void run();
};

#endif /* TASK_EXECUTOR_H */

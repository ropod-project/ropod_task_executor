#ifndef TASK_EXECUTOR_H
#define TASK_EXECUTOR_H

#include <ros/ros.h>
#include <ropod_ros_msgs/Task.h>
#include <ropod_ros_msgs/TaskProgressGOTO.h>
#include <ropod_ros_msgs/Action.h>
#include <ropod_ros_msgs/Waypoint.h>
#include <ropod_ros_msgs/ElevatorRequest.h>
#include <ropod_ros_msgs/ElevatorRequestReply.h>
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
    /**
     * types of Actions
     */
    enum ActionType
    {
        GOTO,
        DOCK,
        UNDOCK,
        REQUEST_ELEVATOR,
        GOTO_ELEVATOR,
        ENTER_ELEVATOR,
        EXIT_ELEVATOR
    };

    /**
     * internal state machine states
     */
    enum State
    {
        INIT,
        DISPATCHING_ACTION,
        EXECUTING_ACTION,
        TASK_DONE
    };

    /**
     * Current state in internal state machine
     */
    State state;

    /**
     * ros private node handle
     */
    ros::NodeHandle nh;

    /**
     * Subscriber for task message
     */
    ros::Subscriber task_sub;

    /**
     * Subscriber for elevator reply to request
     */
    ros::Subscriber elevator_reply_sub;

    /**
     * Subscriber for TaskProgressGOTO messages (from navigation)
     */
    ros::Subscriber task_progress_goto_sub;

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
     * Publisher for sending elevator requests
     */
    ros::Publisher elevator_request_pub;

    /**
     * Publisher for sending TaskProgressGOTO messages (to com mediator)
     */
    ros::Publisher task_progress_goto_pub;

    /**
     * Reply to elevator request
     */
    ropod_ros_msgs::ElevatorRequestReply::Ptr elevator_reply;

    /**
     * Currently executing task
     */
    ropod_ros_msgs::Task::Ptr current_task;

    /**
     * Type of currently executing action
     */
    ActionType current_action_type;

    /**
     * Flag indicating if a task has been received
     */
    bool received_task;

    /**
     * Flag indicating whether an action within the task is being
     * executed currently
     */
    bool action_ongoing;

    /**
     * Index of current action being executed
     */
    int current_action_index;

    /**
     * ID of current action being executed
     */
    std::string current_action_id;

    /**
     * Query ID for current elevator request
     */
    std::string current_elevator_query_id;

    /**
     * callback for reply to elevator request
     */
    void elevatorReplyCallback(const ropod_ros_msgs::ElevatorRequestReply::Ptr &msg);

    /**
     * Subscriber callback for task messages
     */
    void taskCallback(const ropod_ros_msgs::Task::Ptr &msg);

    /**
     * Subscriber callback for task progress messages for GOTO actions
     */
    void taskProgressGOTOCallback(const ropod_ros_msgs::TaskProgressGOTO::Ptr &msg);


    /**
     * Publish a message to request an elevator
     *
     * @param action message of type "REQUEST_ELEVATOR"
     * @param task_id UUID of current task
     * @param cart_type Type of load being carried at the moment
     */
    void requestElevator(const ropod_ros_msgs::Action &action, const std::string &task_id, const std::string &cart_type);

public:
    TaskExecutor();
    virtual ~TaskExecutor();

    /**
     * Main loop; dispatches actions one by one
     */
    void run();
};

#endif /* TASK_EXECUTOR_H */

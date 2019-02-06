#ifndef TASK_EXECUTOR_H
#define TASK_EXECUTOR_H

#include <ros/ros.h>
#include <ropod_ros_msgs/Task.h>
#include <ropod_ros_msgs/TaskProgressGOTO.h>
#include <ropod_ros_msgs/TaskProgressDOCK.h>
#include <ropod_ros_msgs/Status.h>
#include <ropod_ros_msgs/Action.h>
#include <ropod_ros_msgs/Waypoint.h>
#include <ropod_ros_msgs/ElevatorRequest.h>
#include <ropod_ros_msgs/ElevatorRequestReply.h>

#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>

#include <ftsm_base.h>
#include <ropod_task_executor/action_recovery.h>


using namespace ftsm;
/**
 * Executor of tasks sent by fleet management.
 * Also publishes feedback when intermediate actions are complete,
 * and when the task is complete.
 *
 * @author Santosh Thoduka
 */
class TaskExecutor : public FTSMBase
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
     * Current task status
     */
    ropod_ros_msgs::Status task_status;

    /**
     * Flag indicating whether the current action is the last action in the task
     */
    bool last_action;

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
     * Subscriber for TaskProgressDOCK messages (from navigation)
     */
    ros::Subscriber task_progress_dock_sub;

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
     * Publisher for sending TaskProgressDOCK messages (to com mediator)
     */
    ros::Publisher task_progress_dock_pub;

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
     * Flag indicating whether an action within the task failed
     */
    bool action_failed;

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
     * Name of database used to save task state
     */
    std::string db_name;

    /**
     * Name of collection for saving task queue
     */
    std::string collection_name;

    /**
     * Mongodb instance
     */
    mongocxx::instance mongo_instance;

    /**
     * Action recovery object
     */
    ActionRecovery action_recovery;

    /**
     * Last progress message
     */
    ropod_ros_msgs::TaskProgressGOTO::Ptr goto_progress_msg;

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
     * Subscriber callback for task progress messages for DOCK actions
     */
    void taskProgressDOCKCallback(const ropod_ros_msgs::TaskProgressDOCK::Ptr &msg);


    /**
     * Publish a message to request an elevator
     *
     * @param action message of type "REQUEST_ELEVATOR"
     * @param task_id UUID of current task
     * @param cart_type Type of load being carried at the moment
     */
    void requestElevator(const ropod_ros_msgs::Action &action, const std::string &task_id, const std::string &cart_type);

    /**
     * Queue a new task
     */
    void queueTask(const ropod_ros_msgs::Task::Ptr &task, const std::string &status); 

    /**
     * Check and get next task from queue
     */
    bool getNextTask(ropod_ros_msgs::Task::Ptr &task);

    /**
     * Remove a task from the queue
     */
    void removeTask(const std::string &task_id);

    void setCurrentTask(const ropod_ros_msgs::Task::Ptr &msg);
    void setCurrentActionIndex(int index);

    bool retryFailedAction(const ropod_ros_msgs::TaskProgressGOTO::Ptr &msg);

public:
    TaskExecutor();
    virtual ~TaskExecutor();


    std::string init();
    std::string configuring();
    std::string ready();
    std::string running();
    std::string recovering();

};

#endif /* TASK_EXECUTOR_H */

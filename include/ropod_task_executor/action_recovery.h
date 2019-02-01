#ifndef ACTION_RECOVERY_H
#define ACTION_RECOVERY_H

#include <map>
#include <ros/ros.h>
#include <ropod_ros_msgs/Task.h>
#include <ropod_ros_msgs/Action.h>
#include <ropod_ros_msgs/TaskProgressGOTO.h>

class ActionRecovery
{
private:
    enum RecoveryLevel
    {
        RETRY, // simply retry the same action from current state
        RECONFIGURE, // depends on the action (GOTO: replan route)
        REPLAN // Replan task
    };

    struct RecoveryState
    {
        RecoveryLevel level; // current level of recovery
        int num_retries; // how many times this level of recovery has been retried
        double last_recover_time; // last time (unix timestamp) a recovery action was called for this action
    };

    bool recoverGOTOAction(const ropod_ros_msgs::TaskProgressGOTO::Ptr &msg, ropod_ros_msgs::Action &recovery_action, std::map<std::string, RecoveryState>::iterator &it);

    // index of actions for whom recovery has been initiated
    // key: action_id, value: RecoveryState
    std::map<std::string, RecoveryState> recovery_index;
    // index of areas for whom recovery has been initiated
    // key: area_id, value: RecoveryState
    std::map<std::string, RecoveryState> goto_recovery_index;

    ropod_ros_msgs::Task::Ptr current_task;
    int current_action_index;
    int MAX_RETRIES;

public:
    ActionRecovery();
    virtual ~ActionRecovery();

    void setCurrentTask(const ropod_ros_msgs::Task::Ptr &msg);
    void setCurrentActionIndex(int current_action_index);
    void setTaskDone();
    /**
     * Returns an action to recover from a failed GOTO action
     */
    bool recover(const ropod_ros_msgs::TaskProgressGOTO::Ptr &msg, ropod_ros_msgs::Action &recovery_action);
};

#endif /* ACTION_RECOVERY_H */

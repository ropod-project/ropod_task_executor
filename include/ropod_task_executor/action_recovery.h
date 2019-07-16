#ifndef ACTION_RECOVERY_H
#define ACTION_RECOVERY_H

#include <map>
#include <vector>
#include <ros/ros.h>
#include <ropod_ros_msgs/Task.h>
#include <ropod_ros_msgs/Action.h>

class ActionRecovery
{
protected:
    enum RecoveryLevel
    {
        RETRY,
        RECONFIGURE,
        REPLAN
    };

    struct RecoveryState
    {
        RecoveryLevel level; // current level of recovery
        int num_retries; // how many times this level of recovery has been retried
        double last_recover_time; // last time (unix timestamp) a recovery action was called for this action
    };

    // index of actions for whom recovery has been initiated
    // key: action_id, value: RecoveryState
    std::map<std::string, RecoveryState> recovery_index;

    ropod_ros_msgs::Task::Ptr current_task;
    int current_action_index;
    bool received_progress_message;

    // this variable is populated with the recovery
    // action(s) to be performed (if recovery is successful)
    std::vector<ropod_ros_msgs::Action> recovery_actions;
    const static int MAX_RETRIES = 3;

    virtual bool retry() = 0;
    virtual bool reconfigure() = 0;
    virtual bool replan() = 0;
    virtual std::string getFailedActionId() = 0;

public:
    ActionRecovery();
    virtual ~ActionRecovery();

    void setCurrentTask(const ropod_ros_msgs::Task::Ptr &msg);
    void setCurrentActionIndex(int current_action_index);
    void setTaskDone();
    std::vector<ropod_ros_msgs::Action> getRecoveryActions();


    bool recover();
};

#endif /* ACTION_RECOVERY_H */

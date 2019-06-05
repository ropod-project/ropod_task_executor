#ifndef GOTO_RECOVERY_H
#define GOTO_RECOVERY_H

#include <map>
#include <tuple>
#include <vector>
#include <ros/ros.h>
#include <ropod_ros_msgs/Task.h>
#include <ropod_ros_msgs/Action.h>
#include <ropod_ros_msgs/TaskProgressGOTO.h>
#include <actionlib/client/simple_action_client.h>
#include <ropod_ros_msgs/GetPathPlanAction.h>

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

    bool retryGOTOAction(const ropod_ros_msgs::TaskProgressGOTO::Ptr &msg, ropod_ros_msgs::Action &recovery_action, std::map<std::string, RecoveryState>::iterator &it);
    bool reconfigureGOTOAction(const ropod_ros_msgs::TaskProgressGOTO::Ptr &msg, ropod_ros_msgs::Action &recovery_action, std::map<std::string, RecoveryState>::iterator &it);

    /**
     * Returns a flatted list of (area, subarea, subarea id) tuples in the action
     */
    std::vector<std::tuple<std::string, std::string, std::string>> getSubAreaSequence(const ropod_ros_msgs::Action &action);

    // index of actions for whom recovery has been initiated
    // key: action_id, value: RecoveryState
    std::map<std::string, RecoveryState> recovery_index;
    // index of subareas for whom recovery has been initiated
    // key: sub_area_id, value: RecoveryState
    std::map<std::string, RecoveryState> goto_recovery_index;

    ropod_ros_msgs::Task::Ptr current_task;
    int current_action_index;
    int MAX_RETRIES;

    actionlib::SimpleActionClient<ropod_ros_msgs::GetPathPlanAction> path_planner_client;

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

#endif /* GOTO_RECOVERY_H */

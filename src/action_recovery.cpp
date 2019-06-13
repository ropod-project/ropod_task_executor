#include <ropod_task_executor/action_recovery.h>


ActionRecovery::ActionRecovery()
{
}

ActionRecovery::~ActionRecovery()
{
}

void ActionRecovery::setCurrentTask(const ropod_ros_msgs::Task::Ptr &msg)
{
    current_task = msg;
}

void ActionRecovery::setCurrentActionIndex(int current_action_index)
{
    this->current_action_index = current_action_index;
}

void ActionRecovery::setTaskDone()
{
    current_task.reset();
    current_action_index = -1;
    recovery_index.clear();
}

bool ActionRecovery::recover()
{
    if (!received_progress_message)
    {
        ROS_ERROR_STREAM("Action recovery did not receive TaskProgress message, so recovery cannot be performed");
        return false;
    }
    recovery_actions.clear();

    std::string action_id = getFailedActionId();
    auto it = recovery_index.find(action_id);
    if (it == recovery_index.end())
    {
        RecoveryState rs;
        rs.level = RETRY;
        rs.num_retries = 0;
        recovery_index[action_id] = rs;
        it = recovery_index.find(action_id);
    }
    // first look at current recovery level
    // then decide on the next level
    if (it->second.level == RETRY)
    {
        if (it->second.num_retries < MAX_RETRIES)
        {
            // stay at the same level
        }
        else // go to next level
        {
            it->second.level = RECONFIGURE;
            it->second.num_retries = 0;
        }
    }
    else if (it->second.level == RECONFIGURE)
    {
        if (it->second.num_retries < MAX_RETRIES)
        {
            // stay at the same level
        }
        else
        {
            it->second.level = REPLAN;
            it->second.num_retries = 0;
        }
    }
    else if (it->second.level == REPLAN)
    {
        // we should never reach here, because once we've replanned, this action should never be required to recover again
        ROS_WARN_STREAM("Action already at REPLAN recovery level. This should not happen");
    }

    bool status = false;
    // Based on the level decided above, return an appropriate action
    if (it->second.level == RETRY)
    {
        it->second.num_retries++;
        ROS_INFO_STREAM("Currently on recovery level RETRY on retry number " << it->second.num_retries);
        status = retry();
    }
    else if (it->second.level == RECONFIGURE)
    {
        it->second.num_retries++;
        // route plan
        ROS_INFO_STREAM("Currently on recovery level RECONFIGURE on retry number " << it->second.num_retries);
        status = reconfigure();
    }
    else if (it->second.level == REPLAN)
    {
        ROS_INFO_STREAM("Escalating recovery to task planner");
        ROS_WARN_STREAM("Replanning not implemented yet");
        status = false;
    }
    received_progress_message = false;
    return status;
}

std::vector<ropod_ros_msgs::Action> ActionRecovery::getRecoveryActions()
{
    return recovery_actions;
}

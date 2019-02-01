#include <ropod_task_executor/action_recovery.h>

ActionRecovery::ActionRecovery()
{
    // TODO: make this configurable
    MAX_RETRIES = 3;
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
    goto_recovery_index.clear();
}

bool ActionRecovery::recover(const ropod_ros_msgs::TaskProgressGOTO::Ptr &msg, ropod_ros_msgs::Action &recovery_action)
{
    auto it = recovery_index.find(msg->action_id);
    if (it == recovery_index.end())
    {
        RecoveryState rs;
        rs.level = RETRY;
        rs.num_retries = 0;
        recovery_index[msg->action_id] = rs;
        it = recovery_index.find(msg->action_id);
    }
    // first look at current recovery level
    // then decide on the next level
    if (it->second.level == RETRY)
    {
        // TODO: make this configurable
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

    // Based on the level decided above, return an appropriate action
    if (it->second.level == RETRY)
    {
        bool rec_status = recoverGOTOAction(msg, recovery_action, it);
        ROS_INFO_STREAM("Currently on recovery level RETRY on retry number " << it->second.num_retries);
        return rec_status;
    }
    else if (it->second.level == RECONFIGURE)
    {
        recovery_action = current_task->robot_actions[current_action_index];
        it->second.num_retries++;
        // route plan
        ROS_INFO_STREAM("Currently on recovery level RECONFIGURE on retry number " << it->second.num_retries);
        return true;
    }
    else if (it->second.level == REPLAN)
    {
        ROS_INFO_STREAM("Escalating recovery to task planner");
        return false;
    }
}

bool ActionRecovery::recoverGOTOAction(const ropod_ros_msgs::TaskProgressGOTO::Ptr &msg, ropod_ros_msgs::Action &recovery_action, std::map<std::string, RecoveryState>::iterator &it)
{
    recovery_action = current_task->robot_actions[current_action_index];
    int area_index = -1;
    for (int i = 0; i < recovery_action.areas.size(); i++)
    {
        if (recovery_action.areas[i].name == msg->area_name)
        {
            area_index = i;
            break;
        }
    }
    if (area_index != -1)
    {
        auto area_it = goto_recovery_index.find(msg->area_name);
        if (area_it == goto_recovery_index.end())
        {
            RecoveryState rs;
            rs.level = RETRY;
            rs.num_retries = 0;
            goto_recovery_index[msg->area_name] = rs;
            area_it = goto_recovery_index.find(msg->area_name);
        }
        area_it->second.num_retries++;
        it->second.num_retries = area_it->second.num_retries;
        // TODO: make sure this is not erasing the actual action areas
        recovery_action.areas.erase(recovery_action.areas.begin(), recovery_action.areas.begin() + area_index);
        return true;
    }
    return false;
}

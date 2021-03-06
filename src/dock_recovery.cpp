#include <ropod_task_executor/dock_recovery.h>

DOCKRecovery::DOCKRecovery()
{
}

DOCKRecovery::~DOCKRecovery()
{

}

void DOCKRecovery::setProgressMessage(const ropod_ros_msgs::TaskProgressDOCK &msg)
{
    progress_msg = msg;
    received_progress_message = true;
}

bool DOCKRecovery::retry()
{
    // just retry the same action
    ropod_ros_msgs::Action recovery_action = current_task->robot_actions[current_action_index];
    recovery_actions.push_back(recovery_action);
    return true;
}

bool DOCKRecovery::reconfigure()
{
    ROS_WARN_STREAM("DOCK reconfigure recovery not implemented");
    return false;
}

bool DOCKRecovery::replan()
{
    ROS_WARN_STREAM("DOCK replan recovery not implemented");
    return false;
}

std::string DOCKRecovery::getFailedActionId()
{
    return progress_msg.action_id;
}

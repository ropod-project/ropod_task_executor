#include <ropod_task_executor/action_recovery.h>

ActionRecovery::ActionRecovery() : path_planner_client("/get_path_plan")
{
    // TODO: make this configurable
    MAX_RETRIES = 1;
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
        bool rec_status = retryGOTOAction(msg, recovery_action, it);
        ROS_INFO_STREAM("Currently on recovery level RETRY on retry number " << it->second.num_retries);
        return rec_status;
    }
    else if (it->second.level == RECONFIGURE)
    {
        recovery_action = current_task->robot_actions[current_action_index];
        it->second.num_retries++;
        // route plan
        ROS_INFO_STREAM("Currently on recovery level RECONFIGURE on retry number " << it->second.num_retries);
        bool rec_status = reconfigureGOTOAction(msg, recovery_action, it);
        return true;
    }
    else if (it->second.level == REPLAN)
    {
        ROS_INFO_STREAM("Escalating recovery to task planner");
        return false;
    }
}

bool ActionRecovery::retryGOTOAction(const ropod_ros_msgs::TaskProgressGOTO::Ptr &msg, ropod_ros_msgs::Action &recovery_action, std::map<std::string, RecoveryState>::iterator &it)
{
    recovery_action = current_task->robot_actions[current_action_index];
    int area_index = -1;
    int subarea_index = -1;
    // find the area where GOTO action failed
    for (int i = 0; i < recovery_action.areas.size(); i++)
    {
        if (recovery_action.areas[i].name == msg->area_name)
        {
            area_index = i;
            for (int j = 0; j < recovery_action.areas[i].sub_areas.size(); j++)
            {
                if (recovery_action.areas[i].sub_areas[j].name == msg->subarea_name)
                {
                    subarea_index = j;
                    break;
                }
            }
            break;
        }
    }
    if (area_index != -1 && subarea_index != -1)
    {
        auto area_it = goto_recovery_index.find(msg->subarea_name);
        // first time we are attempting recovery for this area, so keep track of it
        if (area_it == goto_recovery_index.end())
        {
            RecoveryState rs;
            rs.level = RETRY;
            rs.num_retries = 0;
            goto_recovery_index[msg->subarea_name] = rs;
            area_it = goto_recovery_index.find(msg->subarea_name);
        }
        area_it->second.num_retries++;
        it->second.num_retries = area_it->second.num_retries;
        // TODO: make sure this is not erasing the actual action areas
        recovery_action.areas[area_index].sub_areas.erase(recovery_action.areas[area_index].sub_areas.begin(), recovery_action.areas[area_index].sub_areas.begin() + subarea_index);

        recovery_action.areas.erase(recovery_action.areas.begin(), recovery_action.areas.begin() + area_index);
        return true;
    }
    else
    {
        ROS_WARN_STREAM("Area " << msg->area_name << "or sub_area " << msg->subarea_name << " not found in current action. Cannot retry");
    }
    return false;
}

bool ActionRecovery::reconfigureGOTOAction(const ropod_ros_msgs::TaskProgressGOTO::Ptr &msg, ropod_ros_msgs::Action &recovery_action, std::map<std::string, RecoveryState>::iterator &it)
{
    recovery_action = current_task->robot_actions[current_action_index];
    int area_index = -1;
    int subarea_index = -1;
    std::vector<std::tuple<std::string, std::string, std::string>> seq = getSubAreaSequence(recovery_action);
    int failed_subarea_index = -1;
    for (int i = 0; i < seq.size(); i++)
    {
        if (std::get<0>(seq[i]) == msg->area_name && std::get<1>(seq[i]) == msg->subarea_name)
        {
            failed_subarea_index = i;
            break;
        }
    }
    if (failed_subarea_index == 0)
    {
        ROS_WARN_STREAM("this is the first subarea.. not sure what to do here");
        return false;
    }
    // find the area where GOTO action failed
    for (int i = 0; i < recovery_action.areas.size(); i++)
    {
        if (recovery_action.areas[i].name == msg->area_name)
        {
            area_index = i;
            for (int j = 0; j < recovery_action.areas[i].sub_areas.size(); j++)
            {
                if (recovery_action.areas[i].sub_areas[j].name == msg->subarea_name)
                {
                    subarea_index = j;
                    break;
                }
            }
            break;
        }
    }
    // this is not the first recovery attempt for this action
    if (area_index != -1 && subarea_index != -1)
    {
        // check if this is the first time we're trying to recover from reaching this
        // particular area
        auto area_it = goto_recovery_index.find(msg->subarea_name);
        // first time, so keep track of it
        if (area_it == goto_recovery_index.end())
        {
            RecoveryState rs;
            rs.level = RECONFIGURE;
            rs.num_retries = 0;
            goto_recovery_index[msg->subarea_name] = rs;
            area_it = goto_recovery_index.find(msg->subarea_name);
        }
        area_it->second.num_retries++;
        it->second.num_retries = area_it->second.num_retries;

        // erase everything up to one area behind current area
        // unless we're at the first area
        //int erase_up_to = std::max(0, area_index - 1);
        //recovery_action.areas.erase(recovery_action.areas.begin(), recovery_action.areas.begin() + erase_up_to);

        bool path_planner_exists = path_planner_client.waitForServer(ros::Duration(2.0));
        if (!path_planner_exists)
        {
            ROS_WARN_STREAM("Path planner action server does not exist. Cannot reconfigure GOTO action");
            return false;
        }
        ropod_ros_msgs::GetPathPlanGoal goal;
        // TODO: this should probably be taken from the start and end floor of the action
        goal.start_floor = recovery_action.areas[area_index].floor_number;
        goal.destination_floor = recovery_action.areas[area_index].floor_number;
        // start from the previous sub area
        goal.start_area = std::get<0>(seq[failed_subarea_index-1]);
        // end at the sub area after the current one (i.e. after the one we failed
        goal.destination_area = std::get<0>(seq[failed_subarea_index+1]);
        goal.start_sub_area = std::get<1>(seq[failed_subarea_index-1]);
        goal.destination_sub_area = std::get<1>(seq[failed_subarea_index+1]);
        // to reach)
        ropod_ros_msgs::BlockedConnection bc;
        // block connection from previous sub area to the failed sub area
        bc.start_id = std::stoi(std::get<2>(seq[failed_subarea_index-1]));
        bc.end_id = std::stoi(std::get<2>(seq[failed_subarea_index]));
        goal.blocked_connections.push_back(bc);

        goal.relax_traffic_rules = true;

        path_planner_client.sendGoal(goal);
        ROS_INFO_STREAM("Waiting for result");
        path_planner_client.waitForResult();
        ROS_INFO_STREAM("Got result");
        ropod_ros_msgs::GetPathPlanResultConstPtr result = path_planner_client.getResult();
        if (path_planner_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO_STREAM("path plan length: " << result->path_plan.areas.size());
            for (int i = 0; i < result->path_plan.areas.size(); i++)
            {
                for (int j = 0; j < result->path_plan.areas[i].sub_areas.size(); j++)
                {
                    ROS_INFO_STREAM(result->path_plan.areas[i].name << " : " << result->path_plan.areas[i].sub_areas[j].name);
                }
                recovery_action.areas.insert(recovery_action.areas.begin(), result->path_plan.areas[i]);
            }
            return true;
        }
        else
        {
            ROS_WARN_STREAM("Path planner not successful. State: " << path_planner_client.getState().toString());
            return false;
        }
    }
    else
    {
        ROS_WARN_STREAM("Area " << msg->area_name << " not found in current action. Cannot reconfigure");
    }
    return false;
}

std::vector<std::tuple<std::string, std::string, std::string>> ActionRecovery::getSubAreaSequence(const ropod_ros_msgs::Action &action)
{
    std::vector<std::tuple<std::string, std::string, std::string>> seq;
    for (int i = 0; i < action.areas.size(); i++)
    {
        for (int j = 0; j < action.areas[i].sub_areas.size(); j++)
        {
            seq.push_back(std::make_tuple(action.areas[i].name, action.areas[i].sub_areas[j].name, action.areas[i].sub_areas[j].id));
        }
    }
    return seq;
}

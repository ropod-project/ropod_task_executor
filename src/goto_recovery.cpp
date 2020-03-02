#include <ropod_task_executor/goto_recovery.h>

GOTORecovery::GOTORecovery() : path_planner_client("get_path_plan")
{
}

GOTORecovery::~GOTORecovery()
{

}

void GOTORecovery::setProgressMessage(const ropod_ros_msgs::TaskProgressGOTO &msg)
{
    progress_msg = msg;
    received_progress_message = true;
}

void GOTORecovery::setTaskDone()
{
    ActionRecovery::setTaskDone();
    goto_recovery_index.clear();
}

void GOTORecovery::setSubActionSuccessful()
{
    goto_recovery_index.clear();
    recovery_index.clear();
}

bool GOTORecovery::retry()
{
    // Here we erase previously achieved areas/subareas up to the failed area/subarea

    ropod_ros_msgs::Action recovery_action = current_task->robot_actions[current_action_index];
    int failed_area_index = -1;
    int failed_subarea_index = -1;
    // find the area where GOTO action failed
    for (int i = 0; i < recovery_action.areas.size(); i++)
    {
        failed_area_index = i;
        bool found = false;
        for (int j = 0; j < recovery_action.areas[i].sub_areas.size(); j++)
        {
            if (recovery_action.areas[i].sub_areas[j].name == progress_msg.subarea_name)
            {
                failed_subarea_index = j;
                found = true;
                break;
            }
        }
        if (found)
        {
            break;
        }
    }
    if (failed_area_index != -1 && failed_subarea_index != -1)
    {
        auto area_it = goto_recovery_index.find(progress_msg.subarea_name);
        // first time we are attempting recovery for this area, so keep track of it
        if (area_it == goto_recovery_index.end())
        {
            RecoveryState rs;
            rs.level = RETRY;
            rs.num_retries = 0;
            goto_recovery_index[progress_msg.subarea_name] = rs;
            area_it = goto_recovery_index.find(progress_msg.subarea_name);
        }
        area_it->second.num_retries++;
        auto it = recovery_index.find(progress_msg.action_id);
        it->second.num_retries = area_it->second.num_retries;
        // TODO: make sure this is not erasing the actual action areas
        recovery_action.areas[failed_area_index].sub_areas.erase(recovery_action.areas[failed_area_index].sub_areas.begin(), recovery_action.areas[failed_area_index].sub_areas.begin() + failed_subarea_index);

        recovery_action.areas.erase(recovery_action.areas.begin(), recovery_action.areas.begin() + failed_area_index);
        recovery_actions.push_back(recovery_action);
        return true;
    }
    else
    {
        ROS_WARN_STREAM("Area " << progress_msg.area_name << "or sub_area " << progress_msg.subarea_name << " not found in current action. Cannot retry");
    }
    return false;
}

bool GOTORecovery::reconfigure()
{
    ropod_ros_msgs::Action recovery_action = current_task->robot_actions[current_action_index];
    // index of area where failure occurred
    int failed_area_index = -1;
    // index of sub_area within area where failure occurred
    int failed_subarea_index = -1;
    // area name, sub_area name, sub_area id
    std::vector<std::tuple<std::string, std::string, std::string>> seq = getSubAreaSequence(recovery_action);
    // index of failed sub_area in a flatted sub_area list (flatted = 1D array of all subareas in the plan)
    int flat_failed_subarea_index = -1;

    int flat_index = 0;
    // find the area where GOTO action failed
    for (int i = 0; i < recovery_action.areas.size(); i++)
    {
        failed_area_index = i;
        bool found = false;
        for (int j = 0; j < recovery_action.areas[i].sub_areas.size(); j++)
        {
            if (recovery_action.areas[i].sub_areas[j].name == progress_msg.subarea_name)
            {
                failed_subarea_index = j;
                flat_failed_subarea_index = flat_index;
                found = true;
                break;
            }
            flat_index++;
        }
        if (found)
        {
            break;
        }
    }

    if (flat_failed_subarea_index == 0)
    {
        ROS_WARN_STREAM("this is the first sub area.. not sure what to do here");
        return false;
    }

    if (failed_area_index != -1 && failed_subarea_index != -1)
    {
        // check if this is the first time we're trying to recover from reaching this
        // particular area
        auto area_it = goto_recovery_index.find(progress_msg.subarea_name);
        // first time, so keep track of it
        if (area_it == goto_recovery_index.end())
        {
            RecoveryState rs;
            rs.level = RECONFIGURE;
            rs.num_retries = 0;
            goto_recovery_index[progress_msg.subarea_name] = rs;
            area_it = goto_recovery_index.find(progress_msg.subarea_name);
        }
        area_it->second.num_retries++;
        auto it = recovery_index.find(progress_msg.action_id);
        it->second.num_retries = area_it->second.num_retries;

        // AX = area X
        // SAXY: sub area Y for area X
        // Start state: (A1: SA11, SA12, A2: SA21, A3: SA31, A4: SA41)
        // Failed to reach: SA21 after having reached SA12
        // End state:   (<new plan>, A3: SA31, A4: SA41)

        // the path planner is invoked to find a new path from SA12 (last successful sub area) to
        // SA31 (sub area after the one we failed to reach)
        // The returned plan includes SA12, some intermediate sub areas, and SA31
        // The intermediate sub areas do not include SA21 since we have added a blocked connection between SA12 an SA21
        // and relaxed traffic rules
        //
        // After receiving the new plan, we delete SA12 and SA31 from it and prepend the remaining intermediate
        // sub areas to the beginning of the existing plan

        bool path_planner_exists = path_planner_client.waitForServer(ros::Duration(2.0));
        if (!path_planner_exists)
        {
            ROS_WARN_STREAM("Path planner action server does not exist. Cannot reconfigure GOTO action");
            return false;
        }
        ropod_ros_msgs::GetPathPlanGoal goal;
        // TODO: this should probably be taken from the start and end floor of the action
        goal.start_floor = recovery_action.areas[failed_area_index].floor_number;
        goal.destination_floor = recovery_action.areas[failed_area_index].floor_number;
        // start from the previous sub area
        goal.start_area = std::get<0>(seq[flat_failed_subarea_index-1]);
        // end at the sub area after the current one (i.e. after the one we failed
        goal.destination_area = std::get<0>(seq[flat_failed_subarea_index+1]);
        goal.start_sub_area = std::get<1>(seq[flat_failed_subarea_index-1]);
        goal.destination_sub_area = std::get<1>(seq[flat_failed_subarea_index+1]);
        // to reach)
        ropod_ros_msgs::BlockedConnection bc;
        // block connection from previous sub area to the failed sub area
        bc.start_id = std::stoi(std::get<2>(seq[flat_failed_subarea_index-1]));
        bc.end_id = std::stoi(std::get<2>(seq[flat_failed_subarea_index]));
        goal.blocked_connections.push_back(bc);

        goal.relax_traffic_rules = true;

        path_planner_client.sendGoal(goal);
        ROS_INFO_STREAM("Waiting for result from path planner");
        path_planner_client.waitForResult();
        ropod_ros_msgs::GetPathPlanResultConstPtr result = path_planner_client.getResult();
        if (path_planner_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO_STREAM("Got new path plan of length: " << result->path_plan.areas.size());

            // delete up to and including the failed area
            recovery_action.areas.erase(recovery_action.areas.begin(), recovery_action.areas.begin() + failed_area_index + 1);

            // delete the first sub area of the next area if it was the destination subarea given to the planner
            if (!recovery_action.areas.empty())
            {
                if (recovery_action.areas[0].sub_areas[0].name == std::get<1>(seq[flat_failed_subarea_index+1]))
                {
                    recovery_action.areas[0].sub_areas.erase(recovery_action.areas[0].sub_areas.begin());
                    // if this area doesn't have any sub areas left, delete the entire area
                    if (recovery_action.areas[0].sub_areas.size() == 0)
                    {
                        recovery_action.areas.erase(recovery_action.areas.begin());
                    }
                }
            }
            int insert_index = 0;
            // insert new plan at the beginning of the current plan
            for (int i = 0; i < result->path_plan.areas.size(); i++)
            {
                if (i == 0)
                {
                    ropod_ros_msgs::Area first_area = result->path_plan.areas[i];
                    // if the start_sub_area is included in the returned plan, delete it
                    if (first_area.sub_areas[0].name == std::get<1>(seq[flat_failed_subarea_index-1]))
                    {
                        first_area.sub_areas.erase(first_area.sub_areas.begin());
                    }
                    // if no sub areas left, delete the area
                    if (!first_area.sub_areas.empty())
                    {
                        recovery_action.areas.insert(recovery_action.areas.begin(), first_area);
                        insert_index++;
                    }
                    continue;
                }
                recovery_action.areas.insert(recovery_action.areas.begin() + insert_index, result->path_plan.areas[i]);
                insert_index++;
            }
            recovery_actions.push_back(recovery_action);
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
        ROS_WARN_STREAM("Area " << progress_msg.area_name << " not found in current action. Cannot reconfigure");
    }
    return false;
}

bool GOTORecovery::replan()
{
    return false;
}

std::vector<std::tuple<std::string, std::string, std::string>> GOTORecovery::getSubAreaSequence(const ropod_ros_msgs::Action &action)
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

std::string GOTORecovery::getFailedActionId()
{
    return progress_msg.action_id;
}


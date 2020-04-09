#include <ropod_task_executor/task_planning_helper.h>
#include <task_planner_ros_wrapper/Predicate.h>
#include <task_planner_ros_wrapper/UpdateKB.h>
#include <task_planner_ros_wrapper/QueryKB.h>
#include <diagnostic_msgs/KeyValue.h>

TaskPlanningHelper::TaskPlanningHelper() :
    nh("~"),
    task_planner_client(nh, "task_planner_action", true),
    path_planner_client(nh, "path_planner_action", true)
{
}

TaskPlanningHelper::~TaskPlanningHelper()
{
}

void TaskPlanningHelper::init()
{
    ROS_INFO_STREAM("[task_executor] waiting for task planner action server...");
    if (!task_planner_client.waitForServer(ros::Duration(5)))
    {
        ROS_WARN_STREAM("[task_executor] Failed to connect to task planner action server");
    }
    ROS_INFO_STREAM("[task_executor] waiting for path planner action server...");
    if (!path_planner_client.waitForServer(ros::Duration(5)))
    {
        ROS_WARN_STREAM("[task_executor] Failed to connect to path planner action server");
    }

    ROS_INFO_STREAM("[task_executor] waiting for update kb server...");
    update_kb_client = nh.serviceClient<task_planner_ros_wrapper::UpdateKB>("update_kb_srv");
    update_kb_client.waitForExistence(ros::Duration(5));
    if (!update_kb_client.exists())
    {
        ROS_WARN_STREAM("[task_executor] Failed to connect to update_kb service");
    }

    query_kb_client = nh.serviceClient<task_planner_ros_wrapper::QueryKB>("query_kb_srv");
    query_kb_client.waitForExistence(ros::Duration(5));
    if (!query_kb_client.exists())
    {
        ROS_WARN_STREAM("[task_executor] Failed to connect to query_kb service");
    }
}

void TaskPlanningHelper::addLocationFluent(const std::string &area_name)
{
    std::string target_location = area_name;
    task_planner_ros_wrapper::Fluent location_fluent;
    location_fluent.name = "location_floor";
    task_planner_ros_wrapper::FluentValue val;
    val.data_type = task_planner_ros_wrapper::FluentValue::INT;
    val.data = "0";
    location_fluent.value = val; // TODO: how to get the target floor number?
    diagnostic_msgs::KeyValue k1;
    k1.key = "loc";
    k1.value = target_location;
    location_fluent.params.push_back(k1);
    task_planner_ros_wrapper::UpdateKB update_kb_msg;
    update_kb_msg.request.fluents_to_add.push_back(location_fluent);
    if (!update_kb_client.call(update_kb_msg))
    {
        ROS_ERROR_STREAM("Could not update KB with floor of target location");
    }
}

ropod_ros_msgs::Area TaskPlanningHelper::getCurrentLocation()
{
    ropod_ros_msgs::Area robot_area;
    task_planner_ros_wrapper::QueryKB query_kb_msg;
    query_kb_msg.request.query_type = task_planner_ros_wrapper::QueryKBRequest::GET_PREDICATE_ASSERTIONS;
    query_kb_msg.request.predicate_name = "robot_at";
    if (!query_kb_client.call(query_kb_msg))
    {
        ROS_ERROR_STREAM("Could not query KB for current location of robot");
        return robot_area;
    }
    if (query_kb_msg.response.predicate_assertions.empty())
    {
        ROS_ERROR_STREAM("Could not get current location of robot from KB");
        return robot_area;
    }
    std::string sub_area;
    bool found_sub_area = false;
    for (int j = 0; j < query_kb_msg.response.predicate_assertions.size(); j++)
    {
        for (int i = 0; i < query_kb_msg.response.predicate_assertions[j].params.size(); i++)
        {
            if (query_kb_msg.response.predicate_assertions[j].params[i].key == "loc")
            {
                sub_area = query_kb_msg.response.predicate_assertions[j].params[i].value;
                found_sub_area = true;
                ROS_INFO_STREAM("robot sub area " << sub_area);
                break;
            }
        }
        if (found_sub_area) break;
    }
    robot_area.name = sub_area.substr(0, sub_area.find_last_of("_"));
    ropod_ros_msgs::SubArea sub_area_msg;
    sub_area_msg.name = sub_area;
    robot_area.sub_areas.push_back(sub_area_msg);
    return robot_area;
}

void TaskPlanningHelper::getTaskPlan(const std::string &target_location, std::vector<ropod_ros_msgs::Action> &planned_actions)
{
    task_planner_ros_wrapper::PlanGoal goal;
    goal.robot_name = "frank";
    goal.task_request.load_type = "mobidik";

    task_planner_ros_wrapper::Predicate goal_pred;
    goal_pred.name = "robot_at";
    diagnostic_msgs::KeyValue p1;
    diagnostic_msgs::KeyValue p2;
    p1.key = "bot";
    p1.value = "frank";
    p2.key = "loc";
    p2.value = target_location;
    goal_pred.params.push_back(p1);
    goal_pred.params.push_back(p2);
    goal.task_goals.push_back(goal_pred);

    task_planner_client.sendGoal(goal);
    bool finished_before_timeout = task_planner_client.waitForResult(ros::Duration(30.0));
    ROS_WARN_STREAM("Finished before timeout " << finished_before_timeout);
    task_planner_ros_wrapper::PlanResult::ConstPtr result = task_planner_client.getResult();
    ROS_WARN_STREAM("Result " << result->plan_found);
    ROS_WARN_STREAM("Result size " << result->actions.size());
    for (int i = 0; i < result->actions.size(); i++)
    {
        planned_actions.push_back(result->actions[i]);
    }

}

void TaskPlanningHelper::getPlanFromCurrentLocation(const ropod_ros_msgs::Action &first_action, std::vector<ropod_ros_msgs::Action> &planned_actions)
{
    if (first_action.areas.empty())
    {
        ROS_ERROR_STREAM("First action in plan received from CCU does not have an area. Cannot make a plan from current robot location.");
        return;
    }
    if (first_action.areas[0].sub_areas.empty())
    {
        ROS_ERROR_STREAM("First action in plan received from CCU does not have a subarea. Cannot make a plan from current robot location");
        return;
    }

    // get current area and sub area
    ropod_ros_msgs::Area robot_area = getCurrentLocation();
    if (robot_area.sub_areas.empty())
    {
        return;
    }
    addLocationFluent(first_action.areas[0].name);
    addLocationFluent(first_action.areas[0].sub_areas[0].name);
    // get task plan to first area in the current plan
    getTaskPlan(first_action.areas[0].name, planned_actions);
    ropod_ros_msgs::GetPathPlanGoal goal;
    int start_floor = 2;
    std::string start_area = robot_area.name;
    std::string start_sub_area = robot_area.sub_areas[0].name;

    int destination_floor = first_action.areas[0].floor_number;
    std::string destination_area = first_action.areas[0].name;
    std::string destination_sub_area = first_action.areas[0].sub_areas[0].name;

    // create path plan for each go to action
    for (int i = 0; i < planned_actions.size(); i++)
    {
        if (planned_actions[i].type == "GOTO")
        {
            goal.destination_floor = planned_actions[i].areas[0].floor_number;
            goal.destination_area = planned_actions[i].areas[0].name;
            goal.destination_sub_area = goal.destination_area + "_LA1"; // TODO: fix this


            goal.start_floor = start_floor;
            goal.start_area = start_area;
            goal.start_sub_area = start_sub_area;
            goal.relax_traffic_rules = false;

            ROS_INFO_STREAM("Planning path from " << goal.start_area << " to " << goal.destination_area);

            path_planner_client.sendGoal(goal);
            path_planner_client.waitForResult();
            ropod_ros_msgs::GetPathPlanResultConstPtr result = path_planner_client.getResult();

            if (path_planner_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                int insert_index = 0;
                for (int j = 0; j < result->path_plan.areas.size(); j++)
                {
                    planned_actions[i].areas.insert(planned_actions[i].areas.begin() + insert_index, result->path_plan.areas[j]);
                    insert_index++;
                }
            }

            start_floor = destination_floor;
            start_area = destination_area;
            start_sub_area = destination_sub_area;
        }
        else if (planned_actions[i].type == "EXIT_ELEVATOR")
        {
            start_area = planned_actions[i].areas[0].name;
            start_sub_area = start_area + "_LA1"; // TODO: fix this! We need to get the sub-area from OSM somehow
            // TODO: set destination level of RIDE_ELEVATOR here
        }
    }
}

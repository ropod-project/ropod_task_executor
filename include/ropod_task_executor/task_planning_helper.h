#ifndef TASK_PLANNING_HELPER_H
#define TASK_PLANNING_HELPER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <ropod_ros_msgs/GetPathPlanAction.h>
#include <ropod_ros_msgs/Area.h>
#include <task_planner_ros_wrapper/PlanAction.h>
#include <task_planner_ros_wrapper/PlanGoal.h>

class TaskPlanningHelper
{
private:

    ros::NodeHandle nh;
    /**
     * Action client for task planning
     */
    actionlib::SimpleActionClient<task_planner_ros_wrapper::PlanAction> task_planner_client;

    /**
     * Action client for path planning
     */
    actionlib::SimpleActionClient<ropod_ros_msgs::GetPathPlanAction> path_planner_client;

    /**
     * Service client for updating the task planning knowledge base
     */
    ros::ServiceClient update_kb_client;

    /**
     * Service client for querying the task planning knowledge base
     */
    ros::ServiceClient query_kb_client;

    void addLocationFluent(const std::string &area_name);
    void getTaskPlan(const std::string &target_location, std::vector<ropod_ros_msgs::Action> &planned_actions);
    ropod_ros_msgs::Area getCurrentLocation();

public:
    TaskPlanningHelper();
    virtual ~TaskPlanningHelper();


    void init();
    void getPlanFromCurrentLocation(const ropod_ros_msgs::Action &first_action, std::vector<ropod_ros_msgs::Action> &planned_actions);
};
#endif

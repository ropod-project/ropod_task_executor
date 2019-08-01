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
#include <ropod_task_executor/action_recovery.h>

class GOTORecovery : public ActionRecovery
{
protected:
    bool retry();
    bool reconfigure();
    bool replan();
    std::string getFailedActionId();

    /**
     * Returns a flatted list of (area, subarea, subarea id) tuples in the action
     */
    std::vector<std::tuple<std::string, std::string, std::string>> getSubAreaSequence(const ropod_ros_msgs::Action &action);

    // index of subareas for whom recovery has been initiated
    // key: sub_area_id, value: RecoveryState
    std::map<std::string, RecoveryState> goto_recovery_index;

    actionlib::SimpleActionClient<ropod_ros_msgs::GetPathPlanAction> path_planner_client;

    ropod_ros_msgs::TaskProgressGOTO progress_msg;

public:
    GOTORecovery();
    virtual ~GOTORecovery();

    void setProgressMessage(const ropod_ros_msgs::TaskProgressGOTO msg);
    void setTaskDone();
    void setSubActionSuccessful();

};

#endif /* GOTO_RECOVERY_H */

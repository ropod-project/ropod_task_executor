#ifndef DOCK_RECOVERY_H
#define DOCK_RECOVERY_H

#include <ros/ros.h>
#include <ropod_ros_msgs/Task.h>
#include <ropod_ros_msgs/Action.h>
#include <ropod_ros_msgs/TaskProgressDOCK.h>
#include <ropod_task_executor/action_recovery.h>

class DOCKRecovery : public ActionRecovery
{
protected:
    bool retry();
    bool reconfigure();
    bool replan();
    std::string getFailedActionId();

    ropod_ros_msgs::TaskProgressDOCK progress_msg;

public:
    DOCKRecovery();
    virtual ~DOCKRecovery();

    void setProgressMessage(const ropod_ros_msgs::TaskProgressDOCK msg);

};

#endif /* DOCK_RECOVERY_H */

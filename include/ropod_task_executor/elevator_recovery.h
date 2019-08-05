#ifndef ELEVATOR_RECOVERY_H
#define ELEVATOR_RECOVERY_H

#include <ros/ros.h>
#include <ropod_ros_msgs/Task.h>
#include <ropod_ros_msgs/Action.h>
#include <ropod_ros_msgs/TaskProgressELEVATOR.h>
#include <ropod_task_executor/action_recovery.h>

class ElevatorRecovery : public ActionRecovery
{
protected:
    bool retry();
    bool reconfigure();
    bool replan();
    std::string getFailedActionId();

    ropod_ros_msgs::TaskProgressELEVATOR progress_msg;

public:
    ElevatorRecovery();
    virtual ~ElevatorRecovery();

    void setProgressMessage(const ropod_ros_msgs::TaskProgressELEVATOR &msg);

};

#endif /* ELEVATOR_RECOVERY_H */

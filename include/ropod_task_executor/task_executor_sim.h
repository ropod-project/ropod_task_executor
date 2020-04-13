#ifndef TASK_EXECUTOR_SIM_H
#define TASK_EXECUTOR_SIM_H

#include "task_executor.h"

/**
 * A extension of the TaskExecutor class to support simulation
 *
 * @author Sushant Chavan
 */
class TaskExecutorSim : public TaskExecutor
{
protected:
    virtual bool initDockClient();
    virtual bool initElevatorClient();

    virtual void startGotoAction(const ropod_ros_msgs::Action& action);
    virtual void startDockAction(const ropod_ros_msgs::Action& action);
    virtual void startUndockAction(const ropod_ros_msgs::Action& action);

public:
    TaskExecutorSim();

    virtual ~TaskExecutorSim(){}
};

#endif /* TASK_EXECUTOR_SIM_H */

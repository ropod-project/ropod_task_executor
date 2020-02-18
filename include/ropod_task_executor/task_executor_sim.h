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
    virtual bool initDockServer();
    virtual bool initElevatorServer();

public:
    TaskExecutorSim();

    virtual ~TaskExecutorSim(){}
};

#endif /* TASK_EXECUTOR_SIM_H */

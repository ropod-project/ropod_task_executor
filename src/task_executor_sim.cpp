#include <ropod_task_executor/task_executor_sim.h>

bool getModeStatus(int argc, char **argv, const std::string modeName="debug_mode")
{
    bool mode = false;
    for(unsigned int i = 0; i < argc; i++)
    {
        std::string argument = std::string(argv[i]);
        std::size_t found = argument.find(modeName);
        if (found != std::string::npos)
        {
            std::string arg_val = argument.substr(argument.find("=") + 1);
            mode = (arg_val == "true");
            break;
        }
    }
    return mode;
}

TaskExecutorSim::TaskExecutorSim() :
    TaskExecutor(true)
{
}

bool TaskExecutorSim::initDockServer()
{
    // Skip initialization of the Docking actions
    return true;
}

bool TaskExecutorSim::initElevatorServer()
{
    // Skip initialization of the Elevator behaviors
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_executor_sim");

    bool debug = getModeStatus(argc, argv, "debug_mode");
    bool sim = getModeStatus(argc, argv, "sim_mode");

    TaskExecutor* task_executor = NULL;
    if (sim)
    {
        ROS_INFO("[Task Executor] Creating a Task Executor simulation object");
        task_executor = new TaskExecutorSim();
    }
    else if (debug)
    {
        ROS_INFO("[Task Executor] Creating a Task Executor debug object");
        task_executor = new TaskExecutor(true);
    }
    else
    {
        ROS_INFO("[Task Executor] Creating a Task Executor object");
        task_executor = new TaskExecutor();
    }

    if (task_executor != NULL)
        ROS_INFO("[Task Executor] Ready.");
    else
        ROS_ERROR("[Task Executor] Could not create a task executor");

    ros::Rate loop_rate(10);
    task_executor->run();
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete task_executor;
    task_executor = NULL;

    return 0;
}

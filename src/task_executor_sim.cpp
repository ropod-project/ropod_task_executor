#include <ropod_task_executor/task_executor_sim.h>

//#define SEND_DUMMY_GOTO_FEEDBACK_AND_RESULT 1
#define SEND_DUMMY_DOCK_FEEDBACK_AND_RESULT 1
#define SEND_DUMMY_UNDOCK_FEEDBACK_AND_RESULT 1

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

bool TaskExecutorSim::initDockClient()
{
    // Skip initialization of the Dock action server
    task_progress_dock_pub = nh.advertise<ropod_ros_msgs::TaskProgressDOCK>("progress_dock_out", 1);
    return true;
}

bool TaskExecutorSim::initElevatorClient()
{
    // Skip initialization of the Elevator behaviors
    return true;
}

void TaskExecutorSim::startGotoAction(const ropod_ros_msgs::Action& action)
{
#ifdef SEND_DUMMY_GOTO_FEEDBACK_AND_RESULT
    ROS_INFO("[TaskExecutorSim] Publishing dummy feedback followed by result for GOTO action ...");
    ropod_ros_msgs::GoToFeedbackPtr feedback(new ropod_ros_msgs::GoToFeedback());
    ropod_ros_msgs::TaskProgressGOTO msg = ropod_ros_msgs::TaskProgressGOTO();
    msg.task_id = current_task->task_id;
    msg.action_id = current_action_id;
    msg.action_type = "GOTO";
    msg.task_status.module_code = ropod_ros_msgs::Status::TASK_EXECUTOR;
    msg.task_status.status_code = ropod_ros_msgs::Status::RUNNING;
    msg.status.status_code  = ropod_ros_msgs::Status::SUCCEEDED;
    feedback->feedback = msg;
    goToFeedbackCb(feedback);

    ropod_ros_msgs::GoToResultPtr result(new ropod_ros_msgs::GoToResult());
    result->success = true;
    goToResultCb(actionlib::SimpleClientGoalState::SUCCEEDED, result);
#else
    TaskExecutor::startGotoAction(action);
#endif
}

void TaskExecutorSim::startDockAction(const ropod_ros_msgs::Action& action)
{
#ifdef SEND_DUMMY_DOCK_FEEDBACK_AND_RESULT
    ROS_INFO("[TaskExecutorSim] Publishing dummy feedback followed by result for DOCK action ...");
    ropod_ros_msgs::DockFeedbackPtr feedback(new ropod_ros_msgs::DockFeedback());
    ropod_ros_msgs::TaskProgressDOCK msg = ropod_ros_msgs::TaskProgressDOCK();
    msg.task_id = current_task->task_id;
    msg.action_id = current_action_id;
    msg.action_type = "DOCK";
    msg.task_status.module_code = ropod_ros_msgs::Status::TASK_EXECUTOR;
    msg.task_status.status_code = ropod_ros_msgs::Status::RUNNING;
    msg.status.status_code  = ropod_ros_msgs::Status::SUCCEEDED;
    feedback->feedback = msg;
    dockFeedbackCb(feedback);

    ropod_ros_msgs::DockResultPtr result(new ropod_ros_msgs::DockResult());
    result->success = true;
    dockResultCb(actionlib::SimpleClientGoalState::SUCCEEDED, result);
#else
    TaskExecutor::startDockAction(action);
#endif
}

void TaskExecutorSim::startUndockAction(const ropod_ros_msgs::Action& action)
{
#ifdef SEND_DUMMY_UNDOCK_FEEDBACK_AND_RESULT
    ROS_INFO("[TaskExecutorSim] Publishing dummy feedback followed by result for UNDOCK action ...");
    ropod_ros_msgs::DockFeedbackPtr feedback(new ropod_ros_msgs::DockFeedback());
    ropod_ros_msgs::TaskProgressDOCK msg = ropod_ros_msgs::TaskProgressDOCK();
    msg.task_id = current_task->task_id;
    msg.action_id = current_action_id;
    msg.action_type = "UNDOCK";
    msg.task_status.module_code = ropod_ros_msgs::Status::TASK_EXECUTOR;
    msg.task_status.status_code = ropod_ros_msgs::Status::RUNNING;
    msg.status.status_code  = ropod_ros_msgs::Status::SUCCEEDED;
    feedback->feedback = msg;
    dockFeedbackCb(feedback);

    ropod_ros_msgs::DockResultPtr result(new ropod_ros_msgs::DockResult());
    result->success = true;
    //dockResultCb(actionlib::SimpleClientGoalState::SUCCEEDED, result);
#else
    TaskExecutor::startUndockAction(action);
#endif
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

#include <ropod_task_executor/task_executor.h>

TaskExecutor::TaskExecutor() :
    nh("~"),
    received_task(false),
    execution_ongoing(false),
    action_ongoing(false),
    current_action_index(-1)
{
    task_sub = nh.subscribe("task", 1, &TaskExecutor::taskCallback, this);
    task_feedback_pub = nh.advertise<ropod_ros_msgs::Task>("feedback", 1);
    action_goto_pub = nh.advertise<ropod_ros_msgs::Action>("go_to", 1);
    action_dock_pub = nh.advertise<ropod_ros_msgs::Action>("dock", 1);
    action_undock_pub = nh.advertise<ropod_ros_msgs::Action>("undock", 1);
}

TaskExecutor::~TaskExecutor()
{
}


void TaskExecutor::taskCallback(const ropod_ros_msgs::Task::Ptr &msg)
{
    if (execution_ongoing)
    {
        ROS_WARN_STREAM("Cancelling previous task");
        // TODO: properly cancel an ongoing task
        execution_ongoing = false;
        action_ongoing = false;
        current_action_index = -1;
    }
    current_task = msg;
    received_task = true;
}

void TaskExecutor::run()
{
    if (!received_task)
    {
        return;
    }
    // if task has been received, is ongoing and the current action is
    // also ongoing, do nothing
    if (received_task && execution_ongoing && action_ongoing)
    {
        return;
    }
    else if (received_task && !execution_ongoing)
    {
        current_action_index = 0;
    }

    if (current_action_index >= current_task->robot_actions.size())
    {
        // we're done with the task
        return;
    }
    else
    {
        execution_ongoing = true;
        ropod_ros_msgs::Action action = current_task->robot_actions[current_action_index];
        if (action.type == "go_to")
        {
            action_goto_pub.publish(action);
            action_ongoing = true;
        }
        else if (action.type == "dock")
        {
            action_dock_pub.publish(action);
            action_ongoing = true;
        }
        else if (action.type == "undock")
        {
            action_dock_pub.publish(action);
            action_ongoing = true;
        }
        else
        {
            ROS_ERROR_STREAM("Got invalid action: " << action.type);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_executor");
    TaskExecutor te;
    ROS_INFO("Ready.");

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        te.run();
        loop_rate.sleep();
    }

    return 0;
}

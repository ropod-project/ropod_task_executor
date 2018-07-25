#include <ropod_task_executor/task_executor.h>
#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc

TaskExecutor::TaskExecutor() :
    state(INIT),
    nh("~"),
    received_task(false),
    action_ongoing(false),
    current_action_index(-1)
{
    task_sub = nh.subscribe("task", 1, &TaskExecutor::taskCallback, this);
    task_feedback_pub = nh.advertise<ropod_ros_msgs::Task>("feedback", 1);
    action_goto_pub = nh.advertise<ropod_ros_msgs::Action>("GOTO", 1);
    action_dock_pub = nh.advertise<ropod_ros_msgs::Action>("DOCK", 1);
    action_undock_pub = nh.advertise<ropod_ros_msgs::Action>("UNDOCK", 1);

    elevator_reply_sub = nh.subscribe("elevator_reply", 1, &TaskExecutor::elevatorReplyCallback, this);
    elevator_request_pub = nh.advertise<ropod_ros_msgs::ElevatorRequest>("elevator_request", 1);

    task_progress_goto_pub = nh.advertise<ropod_ros_msgs::TaskProgressGOTO>("progress_goto_out", 1);
    task_progress_goto_sub = nh.subscribe("progress_goto_in", 1, &TaskExecutor::taskProgressGOTOCallback, this);
}

TaskExecutor::~TaskExecutor()
{
}

void TaskExecutor::run()
{
    if (state == INIT)
    {
        if (received_task)
        {
            ROS_INFO_STREAM("Received new task");
            state = DISPATCHING_ACTION;
            current_action_index = 0;
            current_action_id = "";
        }
        return;
    }
    else if (state == DISPATCHING_ACTION)
    {
        if (current_action_index >= current_task->robot_actions.size())
        {
            state = TASK_DONE;
            return;
        }
        current_action_id = current_task->robot_actions[current_action_index].action_id;
        action_ongoing = true;
        ropod_ros_msgs::Action action = current_task->robot_actions[current_action_index];
        ROS_INFO_STREAM("Dispatching action: " << action.type << " ID: " << action.action_id);
        if (action.type == "GOTO")
        {
            action_goto_pub.publish(action);
            current_action_type = GOTO;
        }
        else if (action.type == "DOCK")
        {
            ROS_WARN_STREAM("DOCK action currently unsupported");
            current_action_index++;
            current_action_id = "";
            return;
        }
        else if (action.type == "UNDOCK")
        {
            ROS_WARN_STREAM("UNDOCK action currently unsupported");
            current_action_index++;
            current_action_id = "";
            return;
        }
        else if (action.type == "REQUEST_ELEVATOR")
        {
            current_action_type = REQUEST_ELEVATOR;
            elevator_reply = nullptr;
            requestElevator(action, current_task->task_id, current_task->cart_type);
        }
        else if (action.type == "ENTER_ELEVATOR")
        {
            ROS_WARN_STREAM("ENTER_ELEVATOR action currently unsupported");
            current_action_index++;
            current_action_id = "";
            return;
        }
        else
        {
            ROS_ERROR_STREAM("Got invalid action: " << action.type);
            ROS_ERROR_STREAM("Stopping task execution");
            action_ongoing = false;
            state = INIT;
            return;
        }
        state = EXECUTING_ACTION;
        return;
    }
    else if (state == EXECUTING_ACTION)
    {
        if (!action_ongoing)
        {
            current_action_index++;
            current_action_id = "";
            state = DISPATCHING_ACTION;
            return;
        }
        if (current_action_type == REQUEST_ELEVATOR)
        {
            if (elevator_reply)
            {
                std::cout << "Received elevator request reply " << std::endl;
                // TODO: make sure we get reply from the correct query
                if (elevator_reply->query_success)
                {
                    std::string wp = elevator_reply->elevator_waypoint;
                    // send go to elevator_reply->waypoint here
                    elevator_reply = nullptr;
                    current_action_type = GOTO_ELEVATOR;
                    action_ongoing = true;
                }
            }
        }
        return;
    }
    else if (state == TASK_DONE)
    {
        received_task = false;
        current_action_index = -1;
        current_action_id = "";
        action_ongoing = false;
        state = INIT;
        std::cout << "Task done! " << std::endl;
        return;
    }
}

void TaskExecutor::requestElevator(const ropod_ros_msgs::Action &action, const std::string &task_id, const std::string &cart_type)
{
    ropod_ros_msgs::ElevatorRequest er;
    boost::uuids::uuid uuid = boost::uuids::random_generator()();
    std::stringstream ss;
    ss << uuid;
    er.query_id = ss.str();
    er.command = "CALL_ELEVATOR";
    er.start_floor = action.start_floor;
    er.goal_floor = action.goal_floor;
    er.task_id = task_id;
    er.load = cart_type;
    elevator_request_pub.publish(er);
}

void TaskExecutor::taskCallback(const ropod_ros_msgs::Task::Ptr &msg)
{
    if (state != INIT)
    {
        ROS_WARN_STREAM("Cancelling previous task");
        // TODO: properly cancel an ongoing task
        action_ongoing = false;
        current_action_index = 0;
        current_action_id = "";
        state = DISPATCHING_ACTION;
    }
    current_task = msg;
    received_task = true;
}

void TaskExecutor::elevatorReplyCallback(const ropod_ros_msgs::ElevatorRequestReply::Ptr &msg)
{
    elevator_reply = msg;
}

void TaskExecutor::taskProgressGOTOCallback(const ropod_ros_msgs::TaskProgressGOTO::Ptr &msg)
{
    msg->task_id = current_task->task_id;
    task_progress_goto_pub.publish(*msg);

    if (msg->status == "reached"  &&
        msg->sequenceNumber == msg->totalNumber &&
        msg->action_id == current_action_id)
    {
        action_ongoing = false;
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

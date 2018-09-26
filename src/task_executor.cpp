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
    task_progress_dock_pub = nh.advertise<ropod_ros_msgs::TaskProgressDOCK>("progress_dock_out", 1);
    task_progress_goto_sub = nh.subscribe("progress_goto_in", 1, &TaskExecutor::taskProgressGOTOCallback, this);
    task_progress_dock_sub = nh.subscribe("progress_dock_in", 1, &TaskExecutor::taskProgressDOCKCallback, this);
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
            task_status.status_code = ropod_ros_msgs::Status::ONGOING;
            state = DISPATCHING_ACTION;
            current_action_index = 0;
            current_action_id = "";
            last_action = false;
        }
        return;
    }
    else if (state == DISPATCHING_ACTION)
    {
        if (current_action_index >= current_task->robot_actions.size())
        {
            state = TASK_DONE;
            task_status.status_code = ropod_ros_msgs::Status::COMPLETED;
            return;
        }

        if (current_action_index == current_task->robot_actions.size())
        {
            last_action = true;
        }

        current_action_id = current_task->robot_actions[current_action_index].action_id;
        action_ongoing = true;
        task_status.status_code = ropod_ros_msgs::Status::ONGOING;
        ropod_ros_msgs::Action action = current_task->robot_actions[current_action_index];
        ROS_INFO_STREAM("Dispatching action: " << action.type << " ID: " << action.action_id);
        if (action.type == "GOTO")
        {
            action_goto_pub.publish(action);
            current_action_type = GOTO;
        }
        else if (action.type == "DOCK")
        {
            action_dock_pub.publish(action);
            current_action_type = DOCK;
        }
        else if (action.type == "UNDOCK")
        {
            action_undock_pub.publish(action);
            current_action_type = UNDOCK;
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
            task_status.status_code = ropod_ros_msgs::Status::FAILED;
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
                std::cout << "Expected query id: " << current_elevator_query_id << " received query id: " << elevator_reply->query_id << std::endl;
                if (elevator_reply->query_success && current_elevator_query_id == elevator_reply->query_id)
                {
                    current_elevator_query_id = "";
                    std::string wp = elevator_reply->elevator_waypoint;

                    ropod_ros_msgs::Action goto_action;

                    boost::uuids::uuid uuid = boost::uuids::random_generator()();
                    std::stringstream ss;
                    ss << uuid;
                    goto_action.action_id = ss.str();

                    goto_action.type = "GOTO";
                    ropod_ros_msgs::Area area;
                    area.name = wp;
                    goto_action.areas.push_back(area);

                    ROS_INFO_STREAM("Dispatching action to elevator waypoint: " << elevator_reply->elevator_waypoint);
                    action_goto_pub.publish(goto_action);
                    elevator_reply = nullptr;

                    current_action_type = GOTO_ELEVATOR;
                    state = EXECUTING_ACTION;
                    task_status.status_code = ropod_ros_msgs::Status::ONGOING;
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
    current_elevator_query_id = er.query_id;
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
        // TODO: send cancel status back to FMS
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
    if (last_action)
    {
        msg->task_status.status_code = ropod_ros_msgs::Status::COMPLETED;
    }
    else
    {
        msg ->task_status.status_code = task_status.status_code;
    }

    task_progress_goto_pub.publish(*msg);

    if (msg->status.status_code == ropod_ros_msgs::Status::REACHED &&
        msg->sequenceNumber == msg->totalNumber &&
        msg->action_id == current_action_id)
    {
        action_ongoing = false;
    }
}


void TaskExecutor::taskProgressDOCKCallback(const ropod_ros_msgs::TaskProgressDOCK::Ptr &msg)
{
    msg->task_id = current_task->task_id;
    if (last_action)
    {
        msg->task_status.status_code = ropod_ros_msgs::Status::COMPLETED;
    }
    else
    {
        msg ->task_status.status_code = task_status.status_code;
    }

    task_progress_dock_pub.publish(*msg);

    if (msg->status.status_code == ropod_ros_msgs::Status::DOCKED &&
        msg->action_id == current_action_id)
    {
        action_ongoing = false;
    }
    else if (msg->status.status_code == ropod_ros_msgs::Status::UNDOCKED &&
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

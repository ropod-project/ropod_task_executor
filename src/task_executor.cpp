#include <ropod_task_executor/task_executor.h>
#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc
#include <ros/serialization.h>
#include <bsoncxx/types.hpp>
#include <bsoncxx/builder/stream/document.hpp>

TaskExecutor::TaskExecutor() :
    FTSMBase("task_executor",
             {"roscore", "route_navigation", "com_mediator", "task_planner", "cart_collector"},
             {{"functional", {{"roscore", "ros/ros_master_monitor"},
                              {"com_mediator", "ros/ros_node_monitor"},
                              {"route_navigation", "ros/ros_node_monitor"},
                              {"task_planner", "none"},
                              {"cart_collector", "none"}}}}),
    state(INIT),
    nh("~"),
    received_task(false),
    action_ongoing(false),
    action_failed(false),
    current_action_index(-1)
{
}

TaskExecutor::~TaskExecutor()
{
}

std::string TaskExecutor::init()
{
    nh.param<std::string>("db_name", db_name, "task_execution");
    nh.param<std::string>("collection_name", collection_name, "queued_tasks");

    task_sub = nh.subscribe("task", 1, &TaskExecutor::taskCallback, this);
    task_feedback_pub = nh.advertise<ropod_ros_msgs::Task>("feedback", 1);
    action_goto_pub = nh.advertise<ropod_ros_msgs::Action>("GOTO", 1);
    action_dock_pub = nh.advertise<ropod_ros_msgs::Action>("DOCK", 1);
    action_undock_pub = nh.advertise<ropod_ros_msgs::Action>("UNDOCK", 1);
    action_wait_for_elevator_pub = nh.advertise<ropod_ros_msgs::Action>("WAIT_FOR_ELEVATOR", 1);
    action_enter_elevator_pub = nh.advertise<ropod_ros_msgs::Action>("ENTER_ELEVATOR", 1);
    action_ride_elevator_pub = nh.advertise<ropod_ros_msgs::Action>("RIDE_ELEVATOR", 1);
    action_exit_elevator_pub = nh.advertise<ropod_ros_msgs::Action>("EXIT_ELEVATOR", 1);

    elevator_reply_sub = nh.subscribe("elevator_reply", 1, &TaskExecutor::elevatorReplyCallback, this);
    elevator_request_pub = nh.advertise<ropod_ros_msgs::ElevatorRequest>("elevator_request", 1);

    task_progress_goto_pub = nh.advertise<ropod_ros_msgs::TaskProgressGOTO>("progress_goto_out", 1);
    task_progress_dock_pub = nh.advertise<ropod_ros_msgs::TaskProgressDOCK>("progress_dock_out", 1);
    task_progress_elevator_pub = nh.advertise<ropod_ros_msgs::TaskProgressELEVATOR>("progress_elevator_out", 1);
    task_progress_goto_sub = nh.subscribe("progress_goto_in", 1, &TaskExecutor::taskProgressGOTOCallback, this);
    task_progress_dock_sub = nh.subscribe("progress_dock_in", 1, &TaskExecutor::taskProgressDOCKCallback, this);
    task_progress_elevator_sub = nh.subscribe("progress_elevator_in", 1, &TaskExecutor::taskProgressElevatorCallback, this);

    task_status.module_code = ropod_ros_msgs::Status::TASK_EXECUTOR;

    return FTSMTransitions::INITIALISED;
}

std::string TaskExecutor::configuring()
{
    return FTSMTransitions::DONE_CONFIGURING;
}

std::string TaskExecutor::ready()
{
    if (received_task)
    {
        ROS_INFO_STREAM("Received new task");
        state = DISPATCHING_ACTION;
        setCurrentActionIndex(0);
        current_action_id = "";
        return FTSMTransitions::RUN;
    }
    else
    {
        ropod_ros_msgs::Task::Ptr new_task;
        bool task_exists = getNextTask(new_task);
        if (task_exists)
        {
            setCurrentTask(new_task);
            received_task = true;

            ROS_INFO_STREAM("Starting new task: " << new_task->task_id);
            task_status.status_code = ropod_ros_msgs::Status::RUNNING;
            state = DISPATCHING_ACTION;
            setCurrentActionIndex(0);
            current_action_id = "";
            last_action = false;
            return FTSMTransitions::RUN;
        }
        return FTSMTransitions::WAIT;
    }
}

std::string TaskExecutor::running()
{
    if (state == DISPATCHING_ACTION)
    {
        if (current_action_index >= current_task->robot_actions.size())
        {
            state = TASK_DONE;
            task_status.status_code = ropod_ros_msgs::Status::SUCCEEDED;
            return FTSMTransitions::WAIT;
        }

        if (current_action_index == current_task->robot_actions.size())
        {
            last_action = true;
        }

        current_action_id = current_task->robot_actions[current_action_index].action_id;
        action_ongoing = true;
        action_failed = false;
        task_status.status_code = ropod_ros_msgs::Status::RUNNING;
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
            requestElevator(action, current_task->task_id, current_task->load_type);
        }
        else if (action.type == "WAIT_FOR_ELEVATOR")
        {
            action.elevator.elevator_id = elevator_reply->elevator_id;
            action.elevator.door_id = elevator_reply->elevator_door_id;
            action_wait_for_elevator_pub.publish(action);
            current_action_type = WAIT_FOR_ELEVATOR;
        }
        else if (action.type == "RIDE_ELEVATOR")
        {
            action_ride_elevator_pub.publish(action);
            current_action_type = RIDE_ELEVATOR;
        }
        else if (action.type == "ENTER_ELEVATOR")
        {
            action_enter_elevator_pub.publish(action);
            current_action_type = ENTER_ELEVATOR;
        }
        else if (action.type == "EXIT_ELEVATOR")
        {
            action_exit_elevator_pub.publish(action);
            current_action_type = EXIT_ELEVATOR;
        }
        else
        {
            ROS_ERROR_STREAM("Got invalid action: " << action.type);
            ROS_ERROR_STREAM("Stopping task execution");
            action_ongoing = false;
            task_status.status_code = ropod_ros_msgs::Status::FAILED;
            state = INIT;
            return FTSMTransitions::DONE;
        }
        state = EXECUTING_ACTION;
        return FTSMTransitions::CONTINUE;
    }
    else if (state == EXECUTING_ACTION)
    {
        if (!action_ongoing)
        {
            setCurrentActionIndex(current_action_index+1);
            current_action_id = "";
            state = DISPATCHING_ACTION;
            return FTSMTransitions::CONTINUE;
        }
        if (action_failed)
        {
            return FTSMTransitions::RECOVER;
        }

        if (current_action_type == REQUEST_ELEVATOR)
        {
            if (elevator_reply)
            {
                std::cout << "Received elevator request reply " << std::endl;
                std::cout << "Expected query id: " << current_elevator_query_id << " received query id: " << elevator_reply->query_id << std::endl;
                if (elevator_reply->query_success && current_elevator_query_id == elevator_reply->query_id)
                {
                    ROS_INFO("Received elevator %d and door %d", elevator_reply->elevator_id, elevator_reply->elevator_door_id);
                    action_ongoing = false;
                }
            }
        }
        return FTSMTransitions::CONTINUE;
    }
    else if (state == TASK_DONE)
    {
        received_task = false;
        setCurrentActionIndex(-1);
        current_action_id = "";
        action_ongoing = false;
        action_failed = false;
        state = INIT;
        std::cout << "Task done! " << std::endl;
        action_recovery.setTaskDone();
        removeTask(current_task->task_id);
        return FTSMTransitions::DONE;
    }
}

std::string TaskExecutor::recovering()
{
    bool success = retryFailedAction(goto_progress_msg);
    if (success)
    {
        action_failed = false;
        return FTSMTransitions::DONE_RECOVERING;
    }
    else
    {
        return FTSMTransitions::FAILED_RECOVERY;
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
        if (msg->task_id != current_task->task_id)
        {
            ROS_WARN_STREAM("Received new task during execution. Adding to queue");
            queueTask(msg, "queued");
        }
        else
        {
            ROS_WARN_STREAM("Received same task_id (" << msg->task_id << ") as currently executing task. Ignoring.");
        }
    }
    else
    {
        queueTask(msg, "active");
        setCurrentTask(msg);
        received_task = true;
    }
}

void TaskExecutor::elevatorReplyCallback(const ropod_ros_msgs::ElevatorRequestReply::Ptr &msg)
{
    elevator_reply = msg;
}

void TaskExecutor::taskProgressGOTOCallback(const ropod_ros_msgs::TaskProgressGOTO::Ptr &msg)
{
    msg->task_id = current_task->task_id;
    if (msg->status.module_code == ropod_ros_msgs::Status::ROUTE_NAVIGATION &&
    		(msg->status.status_code == ropod_ros_msgs::Status::FAILED || msg->status.status_code == ropod_ros_msgs::Status::GOAL_NOT_REACHABLE))
    {
        ROS_ERROR_STREAM("Action failed: " << msg->action_id  << " at area: " << msg->area_name);
        goto_progress_msg = msg;
        action_failed = true;
        return;
    }
    if (last_action)
    {
        msg->task_status.module_code = ropod_ros_msgs::Status::TASK_EXECUTOR;
        msg->task_status.status_code = ropod_ros_msgs::Status::SUCCEEDED;
    }
    else
    {
        msg->task_status.status_code = task_status.status_code;
    }

    task_progress_goto_pub.publish(*msg);

    if (msg->status.module_code == ropod_ros_msgs::Status::ROUTE_NAVIGATION &&
    	msg->status.status_code == ropod_ros_msgs::Status::GOAL_REACHED &&
        msg->sequenceNumber == msg->totalNumber &&
        msg->action_id == current_action_id)
    {
        action_ongoing = false;
    }
}

void TaskExecutor::taskProgressElevatorCallback(const ropod_ros_msgs::TaskProgressELEVATOR::Ptr &msg)
{
    msg->task_id = current_task->task_id;
    if (last_action)
    {
        msg->task_status.module_code = ropod_ros_msgs::Status::TASK_EXECUTOR;
    	msg->task_status.status_code = ropod_ros_msgs::Status::SUCCEEDED;
    }
    else
    {
        msg->task_status.status_code = task_status.status_code;
    }

    task_progress_elevator_pub.publish(*msg);

    if (msg->status.module_code == ropod_ros_msgs::Status::ELEVATOR_ACTION &&
    	msg->status.status_code == ropod_ros_msgs::Status::GOAL_REACHED &&
        msg->action_id == current_action_id)
    {
        action_ongoing = false;
    }
}

void TaskExecutor::queueTask(const ropod_ros_msgs::Task::Ptr &task, const std::string &status)
{
    mongocxx::client client(mongocxx::uri{});
    auto coll = client[db_name][collection_name];
    bsoncxx::builder::stream::document document{};

    uint32_t serial_size = ros::serialization::serializationLength(*task);
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
    ros::serialization::OStream stream(buffer.get(), serial_size);
    ros::serialization::serialize(stream, *task);

    bsoncxx::types::b_binary task_data{bsoncxx::binary_sub_type::k_binary,
                              serial_size,
                              buffer.get()};
    document << "task_id" << task->task_id;
    document << "task_status" << status;
    document << "queue_time" << bsoncxx::types::b_date(std::chrono::system_clock::now());
    document << "task_msg" << task_data;
    coll.insert_one(document.view());
}

bool TaskExecutor::getNextTask(ropod_ros_msgs::Task::Ptr &task)
{
    mongocxx::client client(mongocxx::uri{});
    auto coll = client[db_name][collection_name];
    auto doc = coll.find_one({}); // this will get the earliest inserted document
    if (doc)
    {
        task = boost::make_shared<ropod_ros_msgs::Task>();
        std::string task_id = doc.value().view()["task_id"].get_utf8().value.to_string();
        uint32_t serial_size = doc.value().view()["task_msg"].get_binary().size;
        ros::serialization::IStream stream((uint8_t*)(doc.value().view()["task_msg"].get_binary().bytes), serial_size);
        ros::serialization::Serializer<ropod_ros_msgs::Task>::read(stream, *task);

        // set status of task to active
        coll.update_one(bsoncxx::builder::stream::document{} << "task_id" << task_id << bsoncxx::builder::stream::finalize,
                        bsoncxx::builder::stream::document{} << "$set" << bsoncxx::builder::stream::open_document
                        << "task_status" << "active" << bsoncxx::builder::stream::close_document << bsoncxx::builder::stream::finalize);
        return true;
    }
    return false;
}

void TaskExecutor::removeTask(const std::string &task_id)
{
    mongocxx::client client(mongocxx::uri{});
    auto coll = client[db_name][collection_name];
    coll.delete_one(bsoncxx::builder::stream::document{} << "task_id" << task_id << bsoncxx::builder::stream::finalize);
}

void TaskExecutor::taskProgressDOCKCallback(const ropod_ros_msgs::TaskProgressDOCK::Ptr &msg)
{
    msg->task_id = current_task->task_id;
    if (last_action)
    {
        msg->task_status.module_code = ropod_ros_msgs::Status::TASK_EXECUTOR;
    	msg->task_status.status_code = ropod_ros_msgs::Status::SUCCEEDED;
    }
    else
    {
        msg ->task_status.status_code = task_status.status_code;
    }

    task_progress_dock_pub.publish(*msg);

    if (msg->status.module_code == ropod_ros_msgs::Status::MOBIDIK_COLLECTION &&
    	msg->status.status_code == ropod_ros_msgs::Status::DOCKING_SEQUENCE_SUCCEEDED &&
        msg->action_id == current_action_id)
    {
        action_ongoing = false;
    }
    else if (msg->status.module_code == ropod_ros_msgs::Status::MOBIDIK_COLLECTION &&
    		 msg->status.status_code == ropod_ros_msgs::Status::UNDOCKING_SEQUENCE_SUCCEEDED &&
             msg->action_id == current_action_id)
    {
        action_ongoing = false;
    }
    //TODO we have now more cases
}

bool TaskExecutor::retryFailedAction(const ropod_ros_msgs::TaskProgressGOTO::Ptr &msg)
{
    ropod_ros_msgs::Action recovery_action;
    bool success = action_recovery.recover(msg, recovery_action);
    if (success)
    {
        action_goto_pub.publish(recovery_action);
    }
    return success;
}

void TaskExecutor::setCurrentTask(const ropod_ros_msgs::Task::Ptr &msg)
{
    current_task = msg;
    action_recovery.setCurrentTask(msg);
}

void TaskExecutor::setCurrentActionIndex(int index)
{
    current_action_index = index;
    action_recovery.setCurrentActionIndex(current_action_index);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_executor");
    TaskExecutor te;
    ROS_INFO("Ready.");

    ros::Rate loop_rate(10);
    te.run();
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

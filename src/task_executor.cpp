#include <ropod_task_executor/task_executor.h>
#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc

#include <ros/serialization.h>

#include <bsoncxx/types.hpp>
TaskExecutor::TaskExecutor() :
    state(INIT),
    nh("~"),
    received_task(false),
    action_ongoing(false),
    current_action_index(-1)
{
    nh.param<std::string>("db_name", db_name, "task_execution");
    nh.param<std::string>("collection_name", collection_name, "queued_tasks");

    mongo_client = std::make_shared<mongocxx::client>(mongocxx::uri{});

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
        else
        {
            ropod_ros_msgs::Task::Ptr new_task;
            bool task_exists = getNextTask(new_task);
            if (task_exists)
            {
                current_task = new_task;
                received_task = true;

                ROS_INFO_STREAM("Starting new task: " << new_task->task_id);
                state = DISPATCHING_ACTION;
                current_action_index = 0;
                current_action_id = "";
            }
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
        ROS_WARN_STREAM("Received new task during execution. Adding to queue");
        queueTask(msg, "queued");
    }
    else
    {
        queueTask(msg, "active");
        current_task = msg;
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
    task_progress_goto_pub.publish(*msg);

    if (msg->status == "reached"  &&
        msg->sequenceNumber == msg->totalNumber &&
        msg->action_id == current_action_id)
    {
        action_ongoing = false;
    }
}

void TaskExecutor::queueTask(const ropod_ros_msgs::Task::Ptr &task, const std::string &status)
{
    auto coll = (*mongo_client)[db_name][collection_name];
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
    auto coll = (*mongo_client)[db_name][collection_name];
    auto doc = coll.find_one({}); // this will get the earliest inserted document
    if (doc)
    {
        task = boost::make_shared<ropod_ros_msgs::Task>();
        std::string task_id = doc.value().view()["task_id"].get_utf8().value.to_string();
        uint32_t serial_size = doc.value().view()["task_msg"].get_binary().size;
        ros::serialization::IStream stream((uint8_t*)(doc.value().view()["task_msg"].get_binary().bytes), serial_size);
        ros::serialization::Serializer<ropod_ros_msgs::Task>::read(stream, *task);

        // set status of task to active
        using bsoncxx::builder::stream::document;
        using bsoncxx::builder::stream::open_document;
        using bsoncxx::builder::stream::close_document;
        using bsoncxx::builder::stream::finalize;
        coll.update_one(document{} << "task_id" << task_id << finalize,
                        document{} << "$set" << open_document << "status" << "active" << close_document << finalize);
        return true;
    }
    return false;
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

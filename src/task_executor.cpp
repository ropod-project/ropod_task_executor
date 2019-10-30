#include <ropod_task_executor/task_executor.h>
#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc
#include <ros/serialization.h>
#include <task_planner_ros_wrapper/Predicate.h>
#include <diagnostic_msgs/KeyValue.h>
#include <bsoncxx/types.hpp>
#include <bsoncxx/builder/stream/document.hpp>

typedef actionlib::SimpleActionClient<ropod_ros_msgs::GoToAction> Client;

void printTask(ropod_ros_msgs::Task::Ptr &msg);

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
    goto_client(nh, "goto_action", true),
    dock_client(nh, "dock_action", true),
    nav_elevator_client(nh, "nav_elevator_action", true),
    task_planner_client(nh, "task_planner_action", true),
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

    ROS_INFO_STREAM("[task_executor] waiting for goto action server...");
    if (!goto_client.waitForServer(ros::Duration(5)))
    {
        ROS_INFO_STREAM("[task_executor] Failed to connect to goto action server");
        return FTSMTransitions::INIT_FAILED;
    }
    ROS_INFO_STREAM("[task_executor] waiting for dock action server...");
    if (!dock_client.waitForServer(ros::Duration(5)))
    {
        ROS_INFO_STREAM("[task_executor] Failed to connect to dock action server");
        return FTSMTransitions::INIT_FAILED;
    }
    ROS_INFO_STREAM("[task_executor] waiting for elevator navigation action server...");
    if (!nav_elevator_client.waitForServer(ros::Duration(5)))
    {
        ROS_INFO_STREAM("[task_executor] Failed to connect to elevator nav action server");
        return FTSMTransitions::INIT_FAILED;
    }
    ROS_INFO_STREAM("[task_executor] waiting for task planner action server...");
    if (!task_planner_client.waitForServer(ros::Duration(5)))
    {
        ROS_INFO_STREAM("[task_executor] Failed to connect to task planner action server");
        return FTSMTransitions::INIT_FAILED;
    }
    ROS_INFO_STREAM("[task_executor] Connected to all servers successfully.");

    elevator_reply_sub = nh.subscribe("elevator_reply", 1, &TaskExecutor::elevatorReplyCallback, this);
    elevator_request_pub = nh.advertise<ropod_ros_msgs::ElevatorRequest>("elevator_request", 1);

    task_progress_goto_pub = nh.advertise<ropod_ros_msgs::TaskProgressGOTO>("progress_goto_out", 1);
    task_progress_dock_pub = nh.advertise<ropod_ros_msgs::TaskProgressDOCK>("progress_dock_out", 1);
    task_progress_elevator_pub = nh.advertise<ropod_ros_msgs::TaskProgressELEVATOR>("progress_elevator_out", 1);
    /* task_progress_goto_sub = nh.subscribe("progress_goto_in", 1, &TaskExecutor::taskProgressGOTOCallback, this); */
    /* task_progress_dock_sub = nh.subscribe("progress_dock_in", 1, &TaskExecutor::taskProgressDOCKCallback, this); */
    /* task_progress_elevator_sub = nh.subscribe("progress_elevator_in", 1, &TaskExecutor::taskProgressElevatorCallback, this); */

    task_status.module_code = ropod_ros_msgs::Status::TASK_EXECUTOR;

    std::string transition = checkDependsStatuses();
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
            ropod_ros_msgs::GoToGoal goto_goal; 
            goto_goal.action = action;
            goto_client.sendGoal(
                    goto_goal,
                    boost::bind(&TaskExecutor::goToResultCb, this, _1, _2),
                    Client::SimpleActiveCallback(),
                    boost::bind(&TaskExecutor::goToFeedbackCb, this, _1));
            current_action_type = GOTO;
        }
        else if (action.type == "DOCK")
        {
            ropod_ros_msgs::DockGoal dock_goal; 
            dock_goal.action = action;
            dock_client.sendGoal(
                    dock_goal,
                    boost::bind(&TaskExecutor::dockResultCb, this, _1, _2),
                    Client::SimpleActiveCallback(),
                    boost::bind(&TaskExecutor::dockFeedbackCb, this, _1));
            current_action_type = DOCK;
        }
        else if (action.type == "UNDOCK")
        {
            ropod_ros_msgs::DockGoal dock_goal; 
            dock_goal.action = action;
            dock_client.sendGoal(
                    dock_goal,
                    boost::bind(&TaskExecutor::dockResultCb, this, _1, _2),
                    Client::SimpleActiveCallback(),
                    boost::bind(&TaskExecutor::dockFeedbackCb, this, _1));
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
            ropod_ros_msgs::NavElevatorGoal nav_elevator_goal; 
            nav_elevator_goal.action = action;
            nav_elevator_client.sendGoal(
                    nav_elevator_goal,
                    boost::bind(&TaskExecutor::navElevatorResultCb, this, _1, _2),
                    Client::SimpleActiveCallback(),
                    boost::bind(&TaskExecutor::navElevatorFeedbackCb, this, _1));
            current_action_type = WAIT_FOR_ELEVATOR;
        }
        else if (action.type == "ENTER_ELEVATOR")
        {
            ropod_ros_msgs::NavElevatorGoal nav_elevator_goal; 
            nav_elevator_goal.action = action;
            nav_elevator_client.sendGoal(
                    nav_elevator_goal,
                    boost::bind(&TaskExecutor::navElevatorResultCb, this, _1, _2),
                    Client::SimpleActiveCallback(),
                    boost::bind(&TaskExecutor::navElevatorFeedbackCb, this, _1));
            current_action_type = ENTER_ELEVATOR;
        }
        else if (action.type == "RIDE_ELEVATOR")
        {
            ropod_ros_msgs::NavElevatorGoal nav_elevator_goal; 
            nav_elevator_goal.action = action;
            nav_elevator_client.sendGoal(
                    nav_elevator_goal,
                    boost::bind(&TaskExecutor::navElevatorResultCb, this, _1, _2),
                    Client::SimpleActiveCallback(),
                    boost::bind(&TaskExecutor::navElevatorFeedbackCb, this, _1));
            current_action_type = RIDE_ELEVATOR;
        }
        else if (action.type == "EXIT_ELEVATOR")
        {
            ropod_ros_msgs::NavElevatorGoal nav_elevator_goal; 
            nav_elevator_goal.action = action;
            nav_elevator_client.sendGoal(
                    nav_elevator_goal,
                    boost::bind(&TaskExecutor::navElevatorResultCb, this, _1, _2),
                    Client::SimpleActiveCallback(),
                    boost::bind(&TaskExecutor::navElevatorFeedbackCb, this, _1));
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
        goto_recovery.setTaskDone();
        removeTask(current_task->task_id);
        return FTSMTransitions::DONE;
    }
}

std::string TaskExecutor::recovering()
{
    if (state == EXECUTING_ACTION)
    {
        bool success = recoverFailedAction();
        if (success)
        {
            action_failed = false;
            return FTSMTransitions::DONE_RECOVERING;
        }
        else
        {
            // TODO: at this point we need to inform the FMS
            // with a failed TaskProgress message
            return FTSMTransitions::FAILED_RECOVERY;
        }
    }
    else if (state == INIT)
    {
        ROS_WARN_STREAM("Retrying initialization after 2 seconds");
        ros::Duration(2).sleep();
        return FTSMTransitions::DONE_RECOVERING;
    }
}

std::string TaskExecutor::checkDependsStatuses()
{
    std::vector<std::string> non_functional_components;
    for (MonitorIterator mon_iter = this->depend_statuses.begin(); mon_iter != this->depend_statuses.end(); ++mon_iter)
    {
        for (ComponentIterator comp_iter = mon_iter->second.begin(); comp_iter != mon_iter->second.end(); ++comp_iter)
        {
            for (MonitorSpecIterator mon_spec_iter = comp_iter->second.begin(); mon_spec_iter != comp_iter->second.end(); ++mon_spec_iter)
            {
                if (mon_spec_iter->first == "ros/ros_master_monitor")
                {
                    auto status = bsoncxx::from_json(mon_spec_iter->second);
                    try
                    {
                        bool roscore_status = status.view()["status"].get_bool().value;
                        if (!roscore_status)
                        {
                            non_functional_components.push_back(comp_iter->first);
                        }
                    }
                    catch (std::exception &e)
                    {
                        std::cout << e.what() << std::endl;
                        non_functional_components.push_back(comp_iter->first);
                    }
                }
                else if (mon_spec_iter->first == "ros/ros_node_monitor")
                {
                    auto status = bsoncxx::from_json(mon_spec_iter->second);
                    try
                    {
                        auto node_status = status.view()[comp_iter->first].get_bool().value;
                        if (!node_status)
                        {
                            non_functional_components.push_back(comp_iter->first);
                        }
                    }
                    catch (std::exception &e)
                    {
                        non_functional_components.push_back(comp_iter->first);
                    }
                }
            }
        }
    }

    if (!non_functional_components.empty())
    {
        std::stringstream ss;
        ss << "The following components are non-functional [";
        for (int i = 0; i < non_functional_components.size(); i++)
        {
            if (i != 0) ss << ", ";
            ss << non_functional_components.at(i);
        }
        ss << "]";
        ROS_WARN_STREAM(ss.str());
        return FTSMTransitions::INIT_FAILED;
    }
    else
    {
        return FTSMTransitions::INITIALISED;
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

void TaskExecutor::getPlanFromCurrentLocation(const ropod_ros_msgs::Action &first_action, std::vector<ropod_ros_msgs::Action> &planned_actions)
{
    task_planner_ros_wrapper::PlanGoal goal;
    goal.robot_name = "frank";
    goal.task_request.load_type = "mobidik";
    task_planner_ros_wrapper::Predicate goal_pred;
    goal_pred.name = "robot_at";
    diagnostic_msgs::KeyValue p1;
    diagnostic_msgs::KeyValue p2;
    p1.key = "bot";
    p1.value = "frank";
    p2.key = "loc";
    p2.value = first_action.areas[0].sub_areas[0].name;
    goal_pred.params.push_back(p1);
    goal_pred.params.push_back(p2);
    goal.task_goals.push_back(goal_pred);
    task_planner_client.sendGoal(goal);
    bool finished_before_timeout = task_planner_client.waitForResult(ros::Duration(30.0));
    ROS_WARN_STREAM("Finished before timeout " << finished_before_timeout);
    task_planner_ros_wrapper::PlanResult::ConstPtr result = task_planner_client.getResult();
    ROS_WARN_STREAM("Result " << result->plan_found);
    for (int i = 0; i < result->actions.size(); i++)
    {
        planned_actions.push_back(result->actions[i]);
    }
}

void TaskExecutor::elevatorReplyCallback(const ropod_ros_msgs::ElevatorRequestReply::Ptr &msg)
{
    elevator_reply = msg;
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
        std::vector<ropod_ros_msgs::Action> pre_actions;
        getPlanFromCurrentLocation(msg->robot_actions[0], pre_actions);
        for (int i = pre_actions.size()-1; i >= 0; i--)
        {
            msg->robot_actions.insert(msg->robot_actions.begin(), pre_actions[i]);
        }
        queueTask(msg, "active");
        setCurrentTask(msg);
        received_task = true;
    }
}

void TaskExecutor::goToResultCb(const actionlib::SimpleClientGoalState& state,const ropod_ros_msgs::GoToResultConstPtr& result)
{
    ROS_INFO_STREAM(*result);
    if (result->success)
    {
        action_ongoing = false;
    }
    else
    {
        action_failed = true;
    }
}

void TaskExecutor::goToFeedbackCb(const ropod_ros_msgs::GoToFeedbackConstPtr& feedback)
{
    ropod_ros_msgs::TaskProgressGOTO msg = feedback->feedback;
    msg.task_id = current_task->task_id;
    if (msg.status.module_code == ropod_ros_msgs::Status::ROUTE_NAVIGATION &&
    		(msg.status.status_code == ropod_ros_msgs::Status::FAILED ||
             msg.status.status_code == ropod_ros_msgs::Status::GOAL_NOT_REACHABLE))
    {
        ROS_ERROR_STREAM("GOTO action failed: " << msg.action_id  << " at area: " << msg.area_name);
        goto_progress_msg = msg;
        action_failed = true;
        return;
    }

    // this will reset retry counts at the area/subarea level
    goto_recovery.setSubActionSuccessful();
    if (last_action)
    {
        msg.task_status.module_code = ropod_ros_msgs::Status::TASK_EXECUTOR;
        msg.task_status.status_code = ropod_ros_msgs::Status::SUCCEEDED;
    }
    else
    {
        msg.task_status.status_code = task_status.status_code;
    }

    task_progress_goto_pub.publish(msg);
}

void TaskExecutor::dockResultCb(const actionlib::SimpleClientGoalState& state,const ropod_ros_msgs::DockResultConstPtr& result)
{
    ROS_INFO_STREAM(*result);
    if (result->success)
    {
        action_ongoing = false;
    }
    else
    {
        action_failed = true;
    }
}

void TaskExecutor::dockFeedbackCb(const ropod_ros_msgs::DockFeedbackConstPtr& feedback)
{
    ropod_ros_msgs::TaskProgressDOCK msg = feedback->feedback;
    msg.task_id = current_task->task_id;
    if (msg.status.module_code == ropod_ros_msgs::Status::MOBIDIK_COLLECTION &&
        (msg.status.status_code != ropod_ros_msgs::Status::DOCKING_SEQUENCE_SUCCEEDED &&
         msg.status.status_code != ropod_ros_msgs::Status::UNDOCKING_SEQUENCE_SUCCEEDED &&
         msg.status.status_code != ropod_ros_msgs::Status::SUCCEEDED))
    {
        std::string action_type = (current_action_type == DOCK? "DOCK" : "UNDOCK");
        ROS_ERROR_STREAM(action_type << " action failed: " << msg.action_id);
        dock_progress_msg = msg;
        action_failed = true;
        return;
    }

    if (last_action)
    {
        msg.task_status.module_code = ropod_ros_msgs::Status::TASK_EXECUTOR;
    	msg.task_status.status_code = ropod_ros_msgs::Status::SUCCEEDED;
    }
    else
    {
        msg.task_status.status_code = task_status.status_code;
    }

    task_progress_dock_pub.publish(msg);
    //TODO we have now more cases
}

void TaskExecutor::navElevatorResultCb(const actionlib::SimpleClientGoalState& state,const ropod_ros_msgs::NavElevatorResultConstPtr& result)
{
    ROS_INFO_STREAM(*result);
    if (result->success)
    {
        action_ongoing = false;
    }
    else
    {
        action_failed = true;
    }
}

void TaskExecutor::navElevatorFeedbackCb(const ropod_ros_msgs::NavElevatorFeedbackConstPtr& feedback)
{
    ropod_ros_msgs::TaskProgressELEVATOR msg = feedback->feedback;
    msg.task_id = current_task->task_id;

    if (msg.status.module_code == ropod_ros_msgs::Status::ELEVATOR_ACTION &&
            (msg.status.status_code == ropod_ros_msgs::Status::WAITING_POINT_UNREACHABLE ||
             msg.status.status_code == ropod_ros_msgs::Status::ELEVATOR_WAIT_TIMEOUT ||
             msg.status.status_code == ropod_ros_msgs::Status::ELEVATOR_ENTERING_FAILED ||
             msg.status.status_code == ropod_ros_msgs::Status::ELEVATOR_EXITING_FAILED))
    {
        ROS_ERROR_STREAM("ENTER_ELEVATOR action failed: " << msg.action_id);
        elevator_progress_msg = msg;
        action_failed = true;
        return;
    }

    if (last_action)
    {
        msg.task_status.module_code = ropod_ros_msgs::Status::TASK_EXECUTOR;
    	msg.task_status.status_code = ropod_ros_msgs::Status::SUCCEEDED;
    }
    else
    {
        msg.task_status.status_code = task_status.status_code;
    }

    task_progress_elevator_pub.publish(msg);
}

bool TaskExecutor::recoverFailedAction()
{
    if (current_action_type == GOTO)
    {
        goto_recovery.setProgressMessage(goto_progress_msg);
        bool success = goto_recovery.recover();
        if (success)
        {
            std::vector<ropod_ros_msgs::Action> recovery_actions = goto_recovery.getRecoveryActions();
            if (!recovery_actions.empty())
            {
                ropod_ros_msgs::GoToGoal goto_goal; 
                goto_goal.action = recovery_actions[0];
                goto_client.sendGoal(
                        goto_goal,
                        boost::bind(&TaskExecutor::goToResultCb, this, _1, _2),
                        Client::SimpleActiveCallback(),
                        boost::bind(&TaskExecutor::goToFeedbackCb, this, _1));
            }
        }
        return success;
    }
    else if (current_action_type == DOCK || current_action_type == UNDOCK)
    {
        dock_recovery.setProgressMessage(dock_progress_msg);
        bool success = dock_recovery.recover();
        if (success)
        {
            std::vector<ropod_ros_msgs::Action> recovery_actions = dock_recovery.getRecoveryActions();
            if (!recovery_actions.empty())
            {
                ropod_ros_msgs::DockGoal dock_goal; 
                dock_goal.action = recovery_actions[0];
                dock_client.sendGoal(
                        dock_goal,
                        boost::bind(&TaskExecutor::dockResultCb, this, _1, _2),
                        Client::SimpleActiveCallback(),
                        boost::bind(&TaskExecutor::dockFeedbackCb, this, _1));
                }
        }
        return success;
    }
    else if (current_action_type == WAIT_FOR_ELEVATOR ||
             current_action_type == ENTER_ELEVATOR ||
             current_action_type == RIDE_ELEVATOR ||
             current_action_type == EXIT_ELEVATOR)
    {
        elevator_recovery.setProgressMessage(elevator_progress_msg);
        bool success = elevator_recovery.recover();
        if (success)
        {
            std::vector<ropod_ros_msgs::Action> recovery_actions = elevator_recovery.getRecoveryActions();
            if (!recovery_actions.empty())
            {
                auto insert_it = current_task->robot_actions.begin() + current_action_index;
                // delete current action
                current_task->robot_actions.erase(insert_it);
                // insert recovery actions at current location
                current_task->robot_actions.insert(insert_it, recovery_actions.begin(), recovery_actions.end());
                // change state to DISPATCHING so that the first recovery action will be executed
                // TODO: maybe it's not a good idea to change the state here..
                state = DISPATCHING_ACTION;
            }
        }
        return success;
    }
}

void TaskExecutor::setCurrentTask(const ropod_ros_msgs::Task::Ptr &msg)
{
    current_task = msg;
    // TODO
    goto_recovery.setCurrentTask(msg);
    dock_recovery.setCurrentTask(msg);
    elevator_recovery.setCurrentTask(msg);
}

void TaskExecutor::setCurrentActionIndex(int index)
{
    current_action_index = index;
    // TODO
    goto_recovery.setCurrentActionIndex(current_action_index);
    dock_recovery.setCurrentActionIndex(current_action_index);
    elevator_recovery.setCurrentActionIndex(current_action_index);
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

void printTask(ropod_ros_msgs::Task::Ptr &msg)
{
    for (int i = 0; i < msg->robot_actions.size(); i++)
    {
        std::cout << msg->robot_actions[i].type << std::endl;
    }
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

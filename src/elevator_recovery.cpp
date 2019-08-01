#include <ropod_task_executor/elevator_recovery.h>
#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc

ElevatorRecovery::ElevatorRecovery()
{
}

ElevatorRecovery::~ElevatorRecovery()
{

}

void ElevatorRecovery::setProgressMessage(const ropod_ros_msgs::TaskProgressELEVATOR msg)
{
    progress_msg = msg;
    received_progress_message = true;
}

bool ElevatorRecovery::retry()
{
    if (progress_msg.status.status_code == ropod_ros_msgs::Status::ELEVATOR_ENTERING_FAILED)
    {
        ropod_ros_msgs::Action current_action = current_task->robot_actions[current_action_index];
        ropod_ros_msgs::Action exit_elevator_action;
        exit_elevator_action.type = "EXIT_ELEVATOR";
        boost::uuids::random_generator rg;
        std::stringstream ss;
        ss << rg();
        exit_elevator_action.action_id = ss.str();
        exit_elevator_action.start_floor = current_action.start_floor;
        exit_elevator_action.goal_floor = current_action.goal_floor;
        exit_elevator_action.elevator.elevator_id = -1;
        exit_elevator_action.estimated_duration = -1;


        // TODO: add a time delay to the request
        // i.e. request the elevator after X minutes
        // this will allow the elevator to leave and come back later
        ropod_ros_msgs::Action request_elevator_action;
        request_elevator_action.type = "REQUEST_ELEVATOR";
        ss.str("");
        ss << rg();
        request_elevator_action.action_id = ss.str();
        request_elevator_action.start_floor = current_action.start_floor;
        request_elevator_action.goal_floor = current_action.goal_floor;
        request_elevator_action.elevator.elevator_id = -1;
        request_elevator_action.estimated_duration = -1;

        ropod_ros_msgs::Action wait_elevator_action;
        wait_elevator_action.type = "WAIT_FOR_ELEVATOR";
        ss.str("");
        ss << rg();
        wait_elevator_action.action_id = ss.str();
        wait_elevator_action.start_floor = current_action.start_floor;
        wait_elevator_action.goal_floor = current_action.goal_floor;
        // this will be updated based on the reply to the new request
        wait_elevator_action.elevator.elevator_id = -1;
        wait_elevator_action.estimated_duration = -1;

        recovery_actions.push_back(exit_elevator_action);
        recovery_actions.push_back(request_elevator_action);
        recovery_actions.push_back(wait_elevator_action);
        recovery_actions.push_back(current_action);
    }
    return true;
}

bool ElevatorRecovery::reconfigure()
{
    ROS_WARN_STREAM("Elevator reconfigure recovery not implemented");
    return false;
}

bool ElevatorRecovery::replan()
{
    ROS_WARN_STREAM("Elevator replan recovery not implemented");
    return false;
}

std::string ElevatorRecovery::getFailedActionId()
{
    return progress_msg.action_id;
}

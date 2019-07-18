#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ropod_ros_msgs/TaskProgressGOTO.h>
#include <ropod_ros_msgs/TaskProgressDOCK.h>
#include <ropod_ros_msgs/TaskProgressELEVATOR.h>
#include <ropod_ros_msgs/Action.h>
#include <ropod_ros_msgs/ElevatorRequest.h>
#include <ropod_ros_msgs/ElevatorRequestReply.h>
#include <ropod_ros_msgs/GoToAction.h>
#include <ropod_ros_msgs/NavElevatorAction.h>
#include <ropod_ros_msgs/DockAction.h>
#include <stdlib.h>

ros::Publisher elevator_reply_pub;
bool ask_for_failure = true;
std::shared_ptr<actionlib::SimpleActionServer<ropod_ros_msgs::GoToAction>> goto_server;
std::shared_ptr<actionlib::SimpleActionServer<ropod_ros_msgs::DockAction>> dock_server;
std::shared_ptr<actionlib::SimpleActionServer<ropod_ros_msgs::NavElevatorAction>> nav_elevator_server;

bool dummyGotoHandler(const ropod_ros_msgs::Action msg)
{
    int num_areas = msg.areas.size();
    ROS_INFO_STREAM("Received action: " << msg.action_id << " with " << num_areas << " areas");
    ropod_ros_msgs::TaskProgressGOTO out_msg;
    int num_waypoints = 0;
    for (int i = 0; i < num_areas; i++)
    {
        for (int j = 0; j < msg.areas[i].sub_areas.size(); j++)
        {
            num_waypoints++;
        }
    }

    int sequenceNumber = 0;
    bool failed = false;
    for (int i = 0; i < num_areas; i++)
    {
        for (int j = 0; j < msg.areas[i].sub_areas.size(); j++)
        {
            sequenceNumber++;

            out_msg.action_id = msg.action_id;
            out_msg.action_type = msg.type;
            out_msg.area_name = msg.areas[i].name;
            out_msg.status.domain = ropod_ros_msgs::Status::COMPONENT;
            out_msg.status.module_code = ropod_ros_msgs::Status::ROUTE_NAVIGATION;
            out_msg.status.status_code = ropod_ros_msgs::Status::GOAL_REACHED;
            out_msg.subarea_id = msg.areas[i].sub_areas[j].id;
            out_msg.subarea_name = msg.areas[i].sub_areas[j].name;
            if (ask_for_failure)
            {
                ROS_INFO_STREAM("Action type: GOTO, Area: " << out_msg.area_name << " Subarea: " << out_msg.subarea_name << " (" << out_msg.subarea_id << ")");
                ROS_INFO_STREAM("Success? (y/n, or Y to succeed remaining)");
                char c;
                std::cin >> c;
                if (c != 'y' && c != 'Y')
                {
                    out_msg.status.status_code = ropod_ros_msgs::Status::FAILED;
                    // also sand back GOAL_NOT_REACHABLE ?
                    failed = true;
                    ROS_INFO_STREAM("sub area failed: " << out_msg.subarea_name);
                }
                if (c == 'Y')
                {
                    ask_for_failure = false;
                }
            }
            out_msg.sequenceNumber = sequenceNumber;
            out_msg.totalNumber = num_waypoints;
            ropod_ros_msgs::GoToFeedback goto_feedback;
            goto_feedback.feedback = out_msg;
            goto_server->publishFeedback(goto_feedback);
            if (failed)
            {
                break;
            }
        }
        if (failed)
        {
            break;
        }
    }
    return !failed;
}

bool dummyDockHandler(const ropod_ros_msgs::Action msg)
{
    ROS_INFO_STREAM("Received action: " << msg.action_id << " of type " << msg.type);
    bool success = true;
    ropod_ros_msgs::TaskProgressDOCK out_msg;
    out_msg.action_id = msg.action_id;
    out_msg.action_type = msg.type;
    out_msg.status.domain = ropod_ros_msgs::Status::COMPONENT;
    out_msg.status.module_code = ropod_ros_msgs::Status::MOBIDIK_COLLECTION;
    if (msg.type == "DOCK")
    {
        out_msg.status.status_code = ropod_ros_msgs::Status::DOCKING_SEQUENCE_SUCCEEDED;
    }
    else
    {
        out_msg.status.status_code = ropod_ros_msgs::Status::UNDOCKING_SEQUENCE_SUCCEEDED;
    }

    ROS_INFO_STREAM("Action type: " << msg.type);
    ROS_INFO_STREAM("Success? (y/n)");
    char c;
    std::cin >> c;
    if (c != 'y' && c != 'Y')
    {
        if (msg.type == "DOCK")
        {
            out_msg.status.status_code = ropod_ros_msgs::Status::DOCKING_SEQUENCE_FAILED;
        }
        else
        {
            out_msg.status.status_code = ropod_ros_msgs::Status::UNDOCKING_SEQUENCE_FAILED;
        }
        ROS_INFO_STREAM(msg.type << " failed");
        success = false;
    }
    ropod_ros_msgs::DockFeedback dock_feedback;
    dock_feedback.feedback = out_msg;
    dock_server->publishFeedback(dock_feedback);
    return success;
}

bool dummyWaitForElevatorHandler(const ropod_ros_msgs::Action msg)
{
    ropod_ros_msgs::TaskProgressELEVATOR out_msg;
    out_msg.action_id = msg.action_id;
    out_msg.action_type = msg.type;
    out_msg.status.module_code = ropod_ros_msgs::Status::ELEVATOR_ACTION;
    out_msg.status.domain = ropod_ros_msgs::Status::COMPONENT;

    out_msg.status.status_code = ropod_ros_msgs::Status::WAITING;
    ropod_ros_msgs::NavElevatorFeedback nav_elevator_feedback;
    nav_elevator_feedback.feedback = out_msg;
    nav_elevator_server->publishFeedback(nav_elevator_feedback);
    ros::Duration(1.0).sleep();

    ROS_INFO_STREAM("Action type: WAIT_FOR_ELEVATOR");
    ROS_INFO_STREAM("Succeed in reaching waiting waypoint? (y/n)");
    char c;
    std::cin >> c;
    if (c != 'y' && c != 'Y')
    {
        out_msg.status.status_code = ropod_ros_msgs::Status::WAITING_POINT_UNREACHABLE;
        ROS_INFO_STREAM("WAIT_FOR_ELEVATOR failed");
        return false;
    }
    else
    {
        out_msg.status.status_code = ropod_ros_msgs::Status::WAITING_POINT_REACHED;
    }
    nav_elevator_feedback.feedback = out_msg;
    nav_elevator_server->publishFeedback(nav_elevator_feedback);
    ros::Duration(1.0).sleep();

    ROS_INFO_STREAM("Action type: WAIT_FOR_ELEVATOR");
    ROS_INFO_STREAM("Did door open in time? (y/n)");
    std::cin >> c;
    if (c != 'y' && c != 'Y')
    {
        out_msg.status.status_code = ropod_ros_msgs::Status::ELEVATOR_WAIT_TIMEOUT;
        ROS_INFO_STREAM("WAIT_FOR_ELEVATOR failed");
        return false;
    }
    else
    {
        out_msg.status.status_code = ropod_ros_msgs::Status::DOOR_OPENED;
    }

    nav_elevator_feedback.feedback = out_msg;
    nav_elevator_server->publishFeedback(nav_elevator_feedback);
    ros::Duration(1.0).sleep();

    out_msg.status.status_code = ropod_ros_msgs::Status::GOAL_REACHED;
    nav_elevator_feedback.feedback = out_msg;
    nav_elevator_server->publishFeedback(nav_elevator_feedback);
    ros::Duration(1.0).sleep();
    return true;
}

bool dummyEnterElevatorHandler(const ropod_ros_msgs::Action msg)
{
    bool success = true;
    ropod_ros_msgs::TaskProgressELEVATOR out_msg;
    out_msg.action_id = msg.action_id;
    out_msg.action_type = msg.type;
    out_msg.status.module_code = ropod_ros_msgs::Status::ELEVATOR_ACTION;
    out_msg.status.domain = ropod_ros_msgs::Status::COMPONENT;

    out_msg.status.status_code = ropod_ros_msgs::Status::ENTERING;
    ropod_ros_msgs::NavElevatorFeedback nav_elevator_feedback;
    nav_elevator_feedback.feedback = out_msg;
    nav_elevator_server->publishFeedback(nav_elevator_feedback);
    ros::Duration(1.0).sleep();

    ROS_INFO_STREAM("Action type: ENTER_ELEVATOR");
    ROS_INFO_STREAM("Success? (y/n)");
    char c;
    std::cin >> c;
    if (c != 'y' && c != 'Y')
    {
        out_msg.status.status_code = ropod_ros_msgs::Status::ELEVATOR_ENTERING_FAILED;
        ROS_INFO_STREAM("ENTER_ELEVATOR failed");
        success = false;
    }
    else
    {
        out_msg.status.status_code = ropod_ros_msgs::Status::ENTERED_ELEVATOR;
    }

    nav_elevator_feedback.feedback = out_msg;
    nav_elevator_server->publishFeedback(nav_elevator_feedback);
    ros::Duration(1.0).sleep();
    return success;
}

bool dummyRideElevatorHandler(const ropod_ros_msgs::Action msg)
{
    ropod_ros_msgs::TaskProgressELEVATOR out_msg;
    out_msg.action_id = msg.action_id;
    out_msg.action_type = msg.type;
    out_msg.status.module_code = ropod_ros_msgs::Status::ELEVATOR_ACTION;
    out_msg.status.domain = ropod_ros_msgs::Status::COMPONENT;

    out_msg.status.status_code = ropod_ros_msgs::Status::DESTINATION_FLOOR_REACHED;
    ropod_ros_msgs::NavElevatorFeedback nav_elevator_feedback;
    nav_elevator_feedback.feedback = out_msg;
    nav_elevator_server->publishFeedback(nav_elevator_feedback);
    ros::Duration(1.0).sleep();
    return true;
}

bool dummyExitElevatorHandler(const ropod_ros_msgs::Action msg)
{
    ropod_ros_msgs::TaskProgressELEVATOR out_msg;
    out_msg.action_id = msg.action_id;
    out_msg.action_type = msg.type;
    out_msg.status.module_code = ropod_ros_msgs::Status::ELEVATOR_ACTION;
    out_msg.status.domain = ropod_ros_msgs::Status::COMPONENT;

    out_msg.status.status_code = ropod_ros_msgs::Status::EXITING;
    ropod_ros_msgs::NavElevatorFeedback nav_elevator_feedback;
    nav_elevator_feedback.feedback = out_msg;
    nav_elevator_server->publishFeedback(nav_elevator_feedback);
    ros::Duration(1.0).sleep();

    out_msg.status.status_code = ropod_ros_msgs::Status::EXITED_ELEVATOR;
    nav_elevator_feedback.feedback = out_msg;
    nav_elevator_server->publishFeedback(nav_elevator_feedback);
    ros::Duration(1.0).sleep();

    out_msg.status.status_code = ropod_ros_msgs::Status::GOAL_REACHED;
    nav_elevator_feedback.feedback = out_msg;
    nav_elevator_server->publishFeedback(nav_elevator_feedback);
    ros::Duration(1.0).sleep();
    return true;
}

void gotoActionCb(const ropod_ros_msgs::GoToGoalConstPtr& goal)
{
    bool success = dummyGotoHandler(goal->action);
    ROS_INFO_STREAM("Finished executing action\n");
    ropod_ros_msgs::GoToResult goto_result;
    goto_result.success = success;
    if (goto_result.success)
    {
        goto_server->setSucceeded(goto_result);
    }
    else
    {
        goto_server->setAborted(goto_result);
    }
}

void dockActionCb(const ropod_ros_msgs::DockGoalConstPtr& goal)
{
    bool success = dummyDockHandler(goal->action);
    ROS_INFO_STREAM("Finished executing action\n");
    ropod_ros_msgs::DockResult dock_result;
    dock_result.success = success;
    if (dock_result.success)
    {
        dock_server->setSucceeded(dock_result);
    }
    else
    {
        dock_server->setAborted(dock_result);
    }
}

void navElevatorActionCb(const ropod_ros_msgs::NavElevatorGoalConstPtr& goal)
{
    bool success = true;
    if (goal->action.type == "WAIT_FOR_ELEVATOR")
    {
        success = dummyWaitForElevatorHandler(goal->action);
    }
    else if (goal->action.type == "ENTER_ELEVATOR")
    {
        success = dummyEnterElevatorHandler(goal->action);
    }
    else if (goal->action.type == "RIDE_ELEVATOR")
    {
        success = dummyRideElevatorHandler(goal->action);
    }
    else if (goal->action.type == "EXIT_ELEVATOR")
    {
        success = dummyExitElevatorHandler(goal->action);
    }
    ROS_INFO_STREAM("Finished executing action\n");
    ropod_ros_msgs::NavElevatorResult nav_elevator_result;
    nav_elevator_result.success = success;
    if (nav_elevator_result.success)
    {
        nav_elevator_server->setSucceeded(nav_elevator_result);
    }
    else
    {
        nav_elevator_server->setAborted(nav_elevator_result);
    }
}

void elevatorRequestCallback(const ropod_ros_msgs::ElevatorRequest::Ptr &msg)
{
    // publish reply
    ropod_ros_msgs::ElevatorRequestReply out_msg;
    out_msg.query_id = msg->query_id;
    out_msg.query_success = true;
    out_msg.elevator_id = 0;
    out_msg.elevator_door_id = 1;
    elevator_reply_pub.publish(out_msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "execution_mockup");
    ros::NodeHandle nh("~");

    ros::Subscriber sub_elev_req = nh.subscribe("elevator_request", 1, elevatorRequestCallback);
    elevator_reply_pub = nh.advertise<ropod_ros_msgs::ElevatorRequestReply>("elevator_reply", 1);

    goto_server.reset(
            new actionlib::SimpleActionServer<ropod_ros_msgs::GoToAction>(
                nh, "/ropod/goto", boost::bind(gotoActionCb, _1),false));
    goto_server->start();

    dock_server.reset(
            new actionlib::SimpleActionServer<ropod_ros_msgs::DockAction>(
                nh, "/ropod/dock", boost::bind(dockActionCb, _1),false));
    dock_server->start();

    nav_elevator_server.reset(
            new actionlib::SimpleActionServer<ropod_ros_msgs::NavElevatorAction>(
                nh, "/ropod/take_elevator", boost::bind(navElevatorActionCb, _1),false));
    nav_elevator_server->start();

    ros::spin();
    return 0;
}

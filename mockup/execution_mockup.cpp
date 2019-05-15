#include <ros/ros.h>
#include <ropod_ros_msgs/TaskProgressGOTO.h>
#include <ropod_ros_msgs/TaskProgressDOCK.h>
#include <ropod_ros_msgs/TaskProgressELEVATOR.h>
#include <ropod_ros_msgs/Action.h>
#include <ropod_ros_msgs/ElevatorRequest.h>
#include <ropod_ros_msgs/ElevatorRequestReply.h>
#include <stdlib.h>

ros::Publisher goto_progress_pub;
ros::Publisher dock_progress_pub;
ros::Publisher elevator_progress_pub;
ros::Publisher elevator_reply_pub;
bool ask_for_failure = true;

void DOCKCallback(const ropod_ros_msgs::Action::Ptr &msg)
{
    ROS_INFO_STREAM("Received action: " << msg->action_id << " of type " << msg->type);
    ropod_ros_msgs::TaskProgressDOCK out_msg;
    out_msg.action_id = msg->action_id;
    out_msg.action_type = msg->type;
    out_msg.status.domain = ropod_ros_msgs::Status::COMPONENT;
    out_msg.status.module_code = ropod_ros_msgs::Status::MOBIDIK_COLLECTION;
    out_msg.status.status_code = ropod_ros_msgs::Status::DOCKING_SEQUENCE_SUCCEEDED;
    dock_progress_pub.publish(out_msg);
}

void UNDOCKCallback(const ropod_ros_msgs::Action::Ptr &msg)
{
    ROS_INFO_STREAM("Received action: " << msg->action_id << " of type " << msg->type);
    ropod_ros_msgs::TaskProgressDOCK out_msg;
    out_msg.action_id = msg->action_id;
    out_msg.action_type = msg->type;
    out_msg.status.domain = ropod_ros_msgs::Status::COMPONENT;
    out_msg.status.module_code = ropod_ros_msgs::Status::MOBIDIK_COLLECTION;
    out_msg.status.status_code = ropod_ros_msgs::Status::UNDOCKING_SEQUENCE_SUCCEEDED;
    dock_progress_pub.publish(out_msg);
}
void GOTOCallback(const ropod_ros_msgs::Action::Ptr &msg)
{
    int num_areas = msg->areas.size();
    ROS_INFO_STREAM("Received action: " << msg->action_id << " with " << num_areas << " areas");
    ropod_ros_msgs::TaskProgressGOTO out_msg;
    int num_waypoints = 0;
    for (int i = 0; i < num_areas; i++)
    {
        for (int j = 0; j < msg->areas[i].sub_areas.size(); j++)
        {
            num_waypoints++;
        }
    }

    int sequenceNumber = 0;
    bool failed = false;
    for (int i = 0; i < num_areas; i++)
    {
        for (int j = 0; j < msg->areas[i].sub_areas.size(); j++)
        {
            sequenceNumber++;

            out_msg.action_id = msg->action_id;
            out_msg.action_type = msg->type;
            out_msg.area_name = msg->areas[i].name;
            out_msg.status.domain = ropod_ros_msgs::Status::COMPONENT;
            out_msg.status.module_code = ropod_ros_msgs::Status::ROUTE_NAVIGATION;
            out_msg.status.status_code = ropod_ros_msgs::Status::GOAL_REACHED;
            out_msg.subarea_id = msg->areas[i].sub_areas[j].id;
            out_msg.subarea_name = msg->areas[i].sub_areas[j].name;
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
            goto_progress_pub.publish(out_msg);
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
}

void elevatorCallback(const ropod_ros_msgs::Action::Ptr &msg)
{
    ropod_ros_msgs::TaskProgressELEVATOR out_msg;
    out_msg.action_id = msg->action_id;
    out_msg.action_type = msg->type;
    out_msg.status.module_code = ropod_ros_msgs::Status::ELEVATOR_ACTION;
    out_msg.status.domain = ropod_ros_msgs::Status::COMPONENT;

    out_msg.status.status_code = ropod_ros_msgs::Status::WAITING;
    elevator_progress_pub.publish(out_msg);
    ros::Duration(1.0).sleep();

    out_msg.status.status_code = ropod_ros_msgs::Status::ENTERING;
    elevator_progress_pub.publish(out_msg);
    ros::Duration(1.0).sleep();

    out_msg.status.status_code = ropod_ros_msgs::Status::ELEVATOR_REACHED;
    elevator_progress_pub.publish(out_msg);
    ros::Duration(1.0).sleep();
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
    ros::Subscriber sub1 = nh.subscribe("GOTO", 1, GOTOCallback);
    ros::Subscriber sub2 = nh.subscribe("ELEVATOR", 1, elevatorCallback);
    ros::Subscriber sub3 = nh.subscribe("elevator_request", 1, elevatorRequestCallback);
    ros::Subscriber sub4 = nh.subscribe("DOCK", 1, DOCKCallback);
    ros::Subscriber sub5 = nh.subscribe("UNDOCK", 1, UNDOCKCallback);
    goto_progress_pub = nh.advertise<ropod_ros_msgs::TaskProgressGOTO>("progress_goto", 1);
    dock_progress_pub = nh.advertise<ropod_ros_msgs::TaskProgressDOCK>("progress_dock", 1);
    elevator_progress_pub = nh.advertise<ropod_ros_msgs::TaskProgressELEVATOR>("progress_elevator", 1);
    elevator_reply_pub = nh.advertise<ropod_ros_msgs::ElevatorRequestReply>("elevator_reply", 1);
    ros::spin();
    return 0;
}

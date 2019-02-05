#include <ros/ros.h>
#include <ropod_ros_msgs/TaskProgressGOTO.h>
#include <ropod_ros_msgs/Action.h>
#include <ropod_ros_msgs/ElevatorRequest.h>
#include <ropod_ros_msgs/ElevatorRequestReply.h>
#include <stdlib.h> 

ros::Publisher task_progress_pub;
ros::Publisher elevator_reply_pub;
double wait_time = 0.5;
bool with_failed_actions = false;

void GOTOCallback(const ropod_ros_msgs::Action::Ptr &msg)
{
    int num_areas = msg->areas.size();
    ROS_INFO_STREAM("Received action: " << msg->action_id << " with " << num_areas << " areas");
    ropod_ros_msgs::TaskProgressGOTO out_msg;
    int area_to_fail = 0;
    bool is_action_failed = true;
    if (with_failed_actions)
    {
        int fail = rand() % 3;
        if (fail != 0)
        {
            is_action_failed = true;
        }
        area_to_fail = rand() % num_areas;
    }
    ROS_INFO_STREAM("Is action failed: " << is_action_failed << " area to fail: " << area_to_fail);
    bool failed = false;
    int num_waypoints = 0;
    for (int i = 0; i < num_areas; i++)
    {
        for (int j = 0; j < msg->areas[i].sub_areas.size(); j++)
        {
            num_waypoints++;
        }
    }
    int sequenceNumber = 0;
    for (int i = 0; i < num_areas; i++)
    {
        for (int j = 0; j < msg->areas[i].sub_areas.size(); j++)
        {
            sequenceNumber++;

            out_msg.action_id = msg->action_id;
            out_msg.action_type = msg->type;
            out_msg.area_name = msg->areas[i].name;
            out_msg.status.domain = ropod_ros_msgs::Status::ACTION_FEEDBACK;
            out_msg.status.status_code = ropod_ros_msgs::Status::REACHED;
            out_msg.subarea_id = msg->areas[i].sub_areas[j].id;
            out_msg.subarea_name = msg->areas[i].sub_areas[j].name;
            //if (is_action_failed && i == area_to_fail)
            if (msg->areas[i].sub_areas[j].id == "66")
            {
                out_msg.status.status_code = ropod_ros_msgs::Status::FAILED;
                failed = true;
                ROS_INFO_STREAM("sub area failed: " << out_msg.subarea_name);
            }
            out_msg.sequenceNumber = sequenceNumber;
            out_msg.totalNumber = num_waypoints;
            task_progress_pub.publish(out_msg);
            ros::Duration(wait_time).sleep();
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

void elevatorRequestCallback(const ropod_ros_msgs::ElevatorRequest::Ptr &msg)
{
    // publish reply
    ropod_ros_msgs::ElevatorRequestReply out_msg;
    out_msg.query_id = msg->query_id;
    out_msg.query_success = true;
    out_msg.elevator_id = 0;
    out_msg.elevator_waypoint = "none";
    elevator_reply_pub.publish(out_msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "execution_mockup");
    ros::NodeHandle nh("~");
    nh.param<double>("wait_time", wait_time, 0.5);
    nh.param<bool>("with_failed_actions", with_failed_actions, false);
    ros::Subscriber sub1 = nh.subscribe("GOTO", 1, GOTOCallback);
    ros::Subscriber sub2 = nh.subscribe("elevator_request", 1, elevatorRequestCallback);
    task_progress_pub = nh.advertise<ropod_ros_msgs::TaskProgressGOTO>("progress_goto", 1);
    elevator_reply_pub = nh.advertise<ropod_ros_msgs::ElevatorRequestReply>("elevator_reply", 1);
    ros::spin();
    return 0;
}

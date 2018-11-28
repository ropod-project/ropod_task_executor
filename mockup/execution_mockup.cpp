#include <ros/ros.h>
#include <ropod_ros_msgs/TaskProgressGOTO.h>
#include <ropod_ros_msgs/Action.h>
#include <ropod_ros_msgs/ElevatorRequest.h>
#include <ropod_ros_msgs/ElevatorRequestReply.h>

ros::Publisher task_progress_pub;
ros::Publisher elevator_reply_pub;
double wait_time = 0.5;

void GOTOCallback(const ropod_ros_msgs::Action::Ptr &msg)
{
    int num_areas = msg->areas.size();
    ropod_ros_msgs::TaskProgressGOTO out_msg;
    for (int i = 0; i < num_areas; i++)
    {
        out_msg.action_id = msg->action_id;
        out_msg.action_type = msg->type;
        out_msg.area_name = msg->areas[i].name;
        out_msg.status.domain = ropod_ros_msgs::Status::ACTION_FEEDBACK;
        out_msg.status.status_code = ropod_ros_msgs::Status::REACHED;
        out_msg.sequenceNumber = i+1;
        out_msg.totalNumber = num_areas;
        task_progress_pub.publish(out_msg);
        ros::Duration(wait_time).sleep();
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
    ros::Subscriber sub1 = nh.subscribe("GOTO", 1, GOTOCallback);
    ros::Subscriber sub2 = nh.subscribe("elevator_request", 1, elevatorRequestCallback);
    task_progress_pub = nh.advertise<ropod_ros_msgs::TaskProgressGOTO>("progress_goto", 1);
    elevator_reply_pub = nh.advertise<ropod_ros_msgs::ElevatorRequestReply>("elevator_reply", 1);
    ros::spin();
    return 0;
}

#include "ros/ros.h"
#include "std_msgs/String.h"

/* This callback function is called whenever there's a new message
on the turtles topic */
void turtlesCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
    // ros::init() is required before using any other part of ROS
    // The node's name: receive_turtle
    ros::init(argc, argv, "receive_turtle", ros::init_options::AnonymousName);

    // The node's main access point to communications with ROS
    ros::NodeHandle n;

    /* Subscribes to turtles topic, and passes the messages received
    to turtlesCallback to print out. The message queue limit is 10.
    */
    ros::Subscriber sub = n.subscribe("turtles", 10, turtlesCallback);

    /* ros::spin() will loop and pump callbacks
    until Ctr-C is pressed or the node is shutdown */
    ros::spin();

    return 0;
}

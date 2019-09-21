/*************************************************** 
Author: Huong Minh Luu
Date last updated: 21/09/19 by Minh
Purpose: Subscriber node named receive_turtle
Subscribed topic: /magic_turtles/turtles
****************************************************/
#include "ros/ros.h"
#include "magic_turtles/Turtle.h"

/* This callback function is called whenever there's a new message
on the magic_turtles/turtles topic */
void turtlesCallback(const magic_turtles::Turtle::ConstPtr& msg) {
    ROS_INFO("Received: Turtle %d, quality: %d", msg->index, msg->quality);
}

int main(int argc, char **argv) {
    // ros::init() is required before using any other part of ROS
    // The node's name: receive_turtle
    ros::init(argc, argv, "receive_turtle", ros::init_options::AnonymousName);

    // The node's main access point to communications with ROS
    ros::NodeHandle n;

    /* Subscribes to magic_turtles/turtles topic, and passes the messages received
    to turtlesCallback to print out. The message queue limit is 10.
    */
    ros::Subscriber sub = n.subscribe("magic_turtles/turtles", 10, turtlesCallback);


    /* ros::spin() will loop and pump callbacks
    until Ctr-C is pressed or the node is shutdown */
    ros::spin();

    return 0;
}

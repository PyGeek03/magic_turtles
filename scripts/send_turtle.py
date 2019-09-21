#!/usr/bin/env python
"""
Author: Huong Minh Luu
Date last updated: 21/09/19 by Minh
Purpose: Publisher node named send_turtle, which pulls from a magic hat 5
turtles every second, and sends high quality turtles (quality >= 7) to the
topic /magic_turtles/turtles
Published topic: /magic_turtles/turtles
"""
import rospy
from magic_turtles.msg import Turtle
from random import randrange


def main():
    # publishes to magic_turtles/turtles topic using message type Turtle,
    # limiting number of queued messages to 10
    pub = rospy.Publisher('magic_turtles/turtles', Turtle, queue_size=10)

    # tells rospy the node's name: send_turtle
    rospy.init_node('send_turtle', anonymous=True)

    # pulls out new turtles at the rate of 5 per second
    rate = rospy.Rate(5)

    # counts the number of turtles
    index = 0

    # pulls out turtles until the program is shutdown
    while not rospy.is_shutdown():
        # pulls out a new turtle with quality from 1 to 10
        quality = randrange(1, 11)
        index += 1

        # sends the turtle if it's of high quality
        if quality >= 7:
            turtle = Turtle()
            turtle.index = index
            turtle.quality = quality

            # publishes the turtle to magic_turtles/turtles topic
            pub.publish(turtle)

        # makes sure we only pull out 5 turtles per second
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        # stops execution if Ctrl-C is pressed or the node is shutdown
        pass

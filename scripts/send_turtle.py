"""
Author: Huong Minh Luu
Date last updated: 21/09/19 by Minh
Purpose: Publisher node named send_turtle
Published topic: /magic_turtles/turtles
"""
#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from random import randrange


def main():
    # publishes to turtles topic using message type String,
    # limiting number of queued messages to 10
    pub = rospy.Publisher('turtles', String, queue_size=10)

    # tells rospy the node's name: send_turtle
    rospy.init_node('send_turtle', anonymous=True)

    # pulls out new turtles at the rate of 5 per second
    rate = rospy.Rate(5)

    # counts the number of turtles
    index = 0

    # pulls out turtles until the program is shutdown
    while not rospy.is_shutdown():
        # pulls out a new turtle with quality from 1 to 10
        new_turtle = randrange(1, 11)
        index += 1

        # sends the turtle if it's of high quality
        if new_turtle >= 7:
            # rospy.get_time() ensures the string sent is always unique
            good_turtle = "Turtle {} with quality {}".format(index, new_turtle)

            # publishes the string to turtles topic
            pub.publish(good_turtle)

        # makes sure we only pulls out 5 turtles per second
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        # stops execution if Ctrl-C is pressed or the node is shutdown
        pass

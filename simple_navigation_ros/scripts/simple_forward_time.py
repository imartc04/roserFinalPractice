#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time

def move(velocity_publisher, time):


    print("Timeout", time)

    ## Write here your own code
    vel_msg = Twist()
    vel_msg.linear.x = 0.1
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    if time > 10 :
        vel_msg.angular.z = 0.04
    else:
        vel_msg.angular.z = -0.08

    velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    # Starts a new node
    rospy.init_node('robot_cleaner', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    try:
     timeout =  time.time() + 30
     while time.time() < timeout:
        move(velocity_publisher, timeout - time.time())

        print("Timeout", )

    except rospy.ROSInterruptException:
     pass
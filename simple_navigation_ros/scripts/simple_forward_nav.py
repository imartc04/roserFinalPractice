#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def move():
    rospy.init_node('robot_cleaner', anonymous=True)

    rate = rospy.Rate(5) # 10hz

    # fetch topic_name from the ~private namespace
    # topic_name = rospy.get_param('~topic_name')
    # rospy.loginfo("%s is %s", rospy.resolve_name('~topic_name'), topic_name)
    
    # velocity_publisher = rospy.Publisher(topic_name, Twist, queue_size=10)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    #Receiveing the user's input
    rospy.loginfo(" %s ", "Let's move your robot")

    rospy.loginfo(" %s ", "Introduce your speed")
    speed = float(input())

    rospy.loginfo(" %s ", "Introduce the total distance")
    distance = float(input())

    rospy.loginfo(" %s ", "Foward? True or False: ")
    isForward = input()#True or False

    #Checking if the movement is forward or backwards
    if(isForward):
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)
        
    #Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    

    #Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_distance = 0
    
    #Loop to move the turtle in an specified distance
    while(current_distance < distance):
        rospy.loginfo(" %f   %f ", current_distance, distance)
        #Publish the velocity
        velocity_publisher.publish(vel_msg)
        #Takes actual time to velocity calculus
        t1 = rospy.Time.now().to_sec()
        #Calculates distancePoseStamped
        current_distance = speed*(t1-t0)
    
    #After the loop, stops the robot
    vel_msg.linear.x = 0
    
    #Force the robot to stop
    velocity_publisher.publish(vel_msg)
    
    rospy.loginfo(" %s ", "Ending")
    

if __name__ == '__main__':
    try:
        #Testing our function
        rospy.loginfo(" %s ", "Welcome to simple forward/backwards navigation example")
        move()
    except rospy.ROSInterruptException: 
        pass
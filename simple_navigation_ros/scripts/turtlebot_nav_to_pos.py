#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from tf.transformations import random_quaternion
from tf.transformations import quaternion_from_euler
import time
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatusArray
import random
from simple_navigation_ros.srv import moveSrv,setPointMapSrv

import sys
import yaml

#Variable storing the working state of the node
# 0 : Go through points in the yaml file sequentially
# 1 : Go trough points in the yaml file in a random way
# 2 : Do nothing, wait for new state
#g_workState = 0 

g_pointsMap = ""
g_goalId = 1

g_goalStatusSus = None

g_currentGoals = None

def move(f_strategy):

    f_strategy = f_strategy.mode
    rospy.loginfo("Calling move with strategy ", f_strategy)


    if f_strategy == 2:

        if g_currentGoals != None:
            #Cancel all current goals
            rospy.loginfo("Cancelling all current goals")
            l_pub = rospy.Publisher("move_base/cancel", GoalID, queue_size=10) 

            for i in g_currentGoals.status_list:
                rospy.loginfo("Cancelling goal id ", i.goal_id.id)
                l_pub.publish(i.goal_id)
        else:
            rospy.loginfo("There is no goals to cancel..")

    else:

        #Create publisher to send goal 
        l_goalPub = rospy.Publisher("move_base/goal", MoveBaseActionGoal, queue_size=10)


        #Prepare goal common data
        l_goal = MoveBaseActionGoal()
        l_goal.goal.target_pose.header.frame_id = "map"

        l_goalX = -3
        l_goalY = 4

        #Load goal from file
        rospy.loginfo("----Trying to open ", g_pointsMap)

        l_points = None
        with open(g_pointsMap) as file:
            l_points = yaml.load(file, Loader=yaml.FullLoader)
            rospy.loginfo(l_points)

        rospy.loginfo("Points loaded succesfully from ", g_pointsMap, " file")

        rospy.loginfo("Type of points ...  ", type(l_points) )

        #Set goal list as sequential by default
        l_loopList = [*range(len(l_points))]

        # Work mode 1 : Obtain random permutation of the initial list 
        if f_strategy == 1:
            rospy.loginfo("Shuffling order of way points...")
            random.shuffle(l_loopList) 
            rospy.loginfo("New order of indices to loop", l_loopList)


        rospy.loginfo("First point ", l_points[0])


        l_goalId = 0

        for i in l_loopList:

            l_point = l_points[i]
            l_goal.goal.target_pose.pose.position.x = l_point[0]
            l_goal.goal.target_pose.pose.position.y = l_point[1]


            #0 valued quaternion results in runtime errors
            l_goal.goal.target_pose.pose.orientation.x = 0.#l_point[2]
            l_goal.goal.target_pose.pose.orientation.y = 0. #l_point[3]
            l_goal.goal.target_pose.pose.orientation.z = 0.3 #l_point[4]

            l_goal.goal_id.id = str(l_goalId)

            l_auxTime = rospy.get_rostime()
            l_goal.header.stamp = l_auxTime
            l_goal.goal_id.stamp = l_auxTime

            #Publish goal 
            #l_goalPub.publish(l_goal)

            rospy.loginfo("Publishing goal with id ", l_goalId)

            l_goalId = l_goalId + 1

            #Loop waiting for 
            l_goalPub.publish(l_goal)

            for i in range(3):
                l_goalPub.publish(l_goal)
                time.sleep(0.2)

# f_map : Full path to the .yaml file of the way points
def setPoints(f_map):

    rospy.loginfo("Type of map ", type(f_map.pointMapPath))
    global g_pointsMap
    g_pointsMap = f_map.pointMapPath
   
    rospy.loginfo("New points map is in path ", g_pointsMap)
    return []

def handleGoalStatus(f_goals):
    g_currentGoals = f_goals

    time.sleep(0.5)

    #rospy.loginfo("num current goals ", len(f_goals.status_list) )

    # for i in f_goals.status_list:
    #     rospy.loginfo("Goal with id ", i.goal_id.id)

    return []

def dummy():
    None


def createService():

    #Suscribe to goal status to check individual goal states
    g_goalStatusSus = rospy.Subscriber("/move_base/status", GoalStatusArray, handleGoalStatus)

    rospy.init_node('turtlebot_move_base_marsi')
    l_s1 = rospy.Service('set_map', setPointMapSrv, setPoints)
    l_s2 = rospy.Service('move', moveSrv, move)
    l_s3 = rospy.Service('dummySrv', moveSrv, dummy)

    rospy.loginfo("Ready to command turtlebot")
    rospy.spin()


if __name__ == '__main__':
    try:

        #Create node service
        createService()


    except rospy.ROSInterruptException:
        pass
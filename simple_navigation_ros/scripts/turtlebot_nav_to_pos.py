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
g_simpleGoalPub = None

g_currentGoals = None

g_moveOrder = False

g_currGoalId = 0
g_numCurrGoals = 0

g_points = []
g_sendOrder = []


def move(f_strategy):

    global g_sendOrder
    global g_points
    global g_currGoalId
    global g_numCurrGoals
    global g_moveOrder

    f_strategy = f_strategy.mode
    rospy.loginfo("Calling move with strategy %s ", f_strategy)


    if f_strategy == 2:

        if g_currentGoals != None:
            #Cancel all current goals
            rospy.loginfo("Cancelling all current goals")
            l_pub = rospy.Publisher("move_base/cancel", GoalID, queue_size=10) 

            for i in g_currentGoals.status_list:
                rospy.loginfo("Cancelling goal id %s", i.goal_id.id)
                l_pub.publish(i.goal_id)
        else:
            rospy.loginfo("There is no goals to cancel..")

    else:

        #Create publisher to send goal 
        #l_goalPub = rospy.Publisher("move_base/goal", MoveBaseActionGoal, queue_size=20)

        

        #Load goal from file
        rospy.loginfo("----Trying to open %s", g_pointsMap)

        g_points = None
        with open(g_pointsMap) as file:
            g_points = yaml.load(file, Loader=yaml.FullLoader)
            rospy.loginfo(g_points)

        rospy.loginfo("Points loaded succesfully from %s file", g_pointsMap)

        rospy.loginfo("Type of points ... %s ", type(g_points) )

        #Set goal list as sequential by default
        g_sendOrder = [*range(len(g_points))]

        # Work mode 1 : Obtain random permutation of the initial list 
        if f_strategy == 1:
            rospy.loginfo("Shuffling order of way points...")
            random.shuffle(g_sendOrder) 
            rospy.loginfo("New order of indices to loop %s ", g_sendOrder)


        rospy.loginfo("First point %s", g_points[0])


        g_currGoalId = 0
        g_numCurrGoals = len(g_points)

        g_moveOrder = True


    return []


def publishGoal(f_point, f_goal):

    global g_simpleGoalPub

    l_point = f_point
    f_goal.goal.target_pose.pose.position.x = l_point[0]
    f_goal.goal.target_pose.pose.position.y = l_point[1]


    #0 valued quaternion results in runtime errors
    f_goal.goal.target_pose.pose.orientation.x = l_point[2]
    f_goal.goal.target_pose.pose.orientation.y = l_point[3]
    f_goal.goal.target_pose.pose.orientation.z = l_point[4]

    l_auxTime = rospy.get_rostime()
    f_goal.header.stamp = l_auxTime
    f_goal.goal_id.stamp = l_auxTime


    g_simpleGoalPub.publish(f_goal.goal.target_pose)



# f_map : Full path to the .yaml file of the way points
def setPoints(f_map):

    #rospy.loginfo("Type of map ", type(f_map.pointMapPath))
    global g_pointsMap
    g_pointsMap = f_map.pointMapPath
   
    rospy.loginfo("New points map is in path %s ", g_pointsMap)
    return []

def handleGoalStatus(f_goals):

    global g_moveOrder
    global g_sendOrder
    global g_points
    global g_currGoalId
    global g_numCurrGoals
    global g_goalCanceller

    g_currentGoals = f_goals.status_list


    rospy.loginfo("num current goals %s ", len(f_goals.status_list) )

    for i in f_goals.status_list:
        rospy.loginfo("Goal with id %s, status %s", i.goal_id.id, i.status)


    #Check if we are going to move
    if g_moveOrder:

        print("Move order")   
        if g_currGoalId < g_numCurrGoals:

            #Move to next index only when all current goals are done
            l_currGoalsDone = True

            for i in g_currentGoals:
                if i.status==1:
                    l_currGoalsDone = False
                else:
                    g_goalCanceller.publish(i.goal_id)
                    #print("Cancelling goal with id ", i.goal_id)

            if l_currGoalsDone:

                #Prepare goal common data
                l_goal = MoveBaseActionGoal()
                l_goal.goal.target_pose.header.frame_id = "map"
                l_goal.goal.target_pose.header.seq = g_currGoalId

                l_goal.goal_id.id = str(g_currGoalId)

                #Publish goal with position info
                publishGoal(g_points[g_sendOrder[g_currGoalId]], l_goal)
        else:
            g_moveOrder = False

    time.sleep(0.1)
    return []

def dummy():
    None


def createService():

    global g_simpleGoalPub
    global g_goalCanceller

    #Suscribe to goal status to check individual goal states
    g_goalStatusSus = rospy.Subscriber("/move_base/status", GoalStatusArray, handleGoalStatus)

    g_simpleGoalPub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=20)
    g_goalCanceller = rospy.Publisher("move_base/cancel", GoalID, queue_size=10) 

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
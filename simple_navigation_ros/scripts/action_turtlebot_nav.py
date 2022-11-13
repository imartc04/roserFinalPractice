#!/usr/bin/env python


# license removed for brevity

import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from simple_navigation_ros.srv import moveSrv,setPointMapSrv
import yaml
import random

g_actionClient = None
g_pointsMap = ""
g_cancelGoals = False

def getWayPointsFromFile():
    global g_pointsMap
    l_points = None
    with open(g_pointsMap) as file:
        l_points = yaml.load(file, Loader=yaml.FullLoader)
        rospy.loginfo(l_points)

    rospy.loginfo("Points loaded succesfully from %s file", g_pointsMap)
    return l_points



def moveThroughWayPoints(f_mode):

    global g_actionClient

    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"

    #Obtain way points from the file 
    l_points = getWayPointsFromFile()

    l_loopIds = [*range(len(l_points))]

    #Set random order if needed
    if f_mode == 1:
        random.shuffle(l_loopIds) 


    #Go over all the points 
    l_ctr = 0
    for i in l_loopIds:

        if g_cancelGoals:
            rospy.loginfo("Goals have been canceled")
            break

        else:
            rospy.loginfo("Going point number  %s", l_ctr)
            l_point = l_points[i]

            goal.target_pose.header.stamp = rospy.Time.now()

            # Set position 
            goal.target_pose.pose.position.x = l_point[0]
            goal.target_pose.pose.position.y = l_point[1]

            #Set orientation
            goal.target_pose.pose.orientation.x = l_point[2]
            goal.target_pose.pose.orientation.y = l_point[3]
            goal.target_pose.pose.orientation.z = l_point[4]
            goal.target_pose.pose.orientation.w = l_point[5]

            # Sends the goal to the action server.
            g_actionClient.send_goal(goal)

            # Waits for the server to finish performing the action.
            wait = g_actionClient.wait_for_result()

            rospy.loginfo("WayPoint reached")

            l_ctr = l_ctr + 1

            # # If the result doesn't arrive, assume the Server is not available
            # if not wait:
            #     rospy.logerr("Action server not available!")
            #     rospy.signal_shutdown("Action server not available!")
            # else:
            # # Result of executing the action
            #     return g_actionClient.get_result()   

    rospy.loginfo("Route through way points done")

def move(f_mode):

    global g_actionClient
    global g_cancelGoals

    if f_mode.mode == 2: 

        #Cancel all current operations
        g_actionClient.cancel_all_goals()
        #g_actionClient.cancel_goals_at_and_before_time(rospy.get_rostime())

        rospy.loginfo("Cancelling all goals")

        g_cancelGoals = True

    else:
        rospy.loginfo("Moving to way points")
        g_cancelGoals = False
        moveThroughWayPoints(f_mode.mode)

    return []


def setWayPoints(f_map):

    global g_pointsMap
    g_pointsMap = f_map.pointMapPath
   
    rospy.loginfo("New points map is in path %s ", g_pointsMap)
    return []

def initialize():

    rospy.init_node('marsi_nav')

    global g_actionClient
    g_actionClient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    g_actionClient.wait_for_server()

    l_s1 = rospy.Service('marsi_nav_set_waypoints', setPointMapSrv, setWayPoints)
    l_s2 = rospy.Service('marsi_nav_move', moveSrv, move)

    rospy.spin()

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        initialize()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
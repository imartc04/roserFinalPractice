#!/usr/bin/env python


# license removed for brevity

import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from simple_navigation_ros.srv import moveSrv,setPointMapSrv, setInitPoseSrv
from geometry_msgs.msg import PoseWithCovarianceStamped

import yaml
import random
import time

g_actionClient = None
g_pointsMap = ""
g_cancelGoals = False
g_initPosePub = None

#g_initPoseReq = None

def getWayPointsFromFile():
    global g_pointsMap
    l_points = {}

    if g_pointsMap != "":
        with open(g_pointsMap) as file:
            l_points = yaml.load(file, Loader=yaml.FullLoader)
            rospy.loginfo(l_points)

        rospy.loginfo("Points loaded succesfully from %s file", g_pointsMap)
    else:
        print("There is no way points file. Set it through service")
    
    return l_points



def moveThroughWayPoints(f_mode):

    global g_actionClient
    global g_cancelGoals


    #Clear all possible goals before begin
    g_actionClient.cancel_all_goals()
    g_cancelGoals = True
    time.sleep(0.2)
    g_cancelGoals = False


    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"

    #Obtain way points from the file 
    l_points = getWayPointsFromFile()

    if len(l_points)>0:

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

    else:
        print("No points to move through!!! Set waypoints file with service")

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

def setInitPose(f_req):
    global g_initPosePub
    # global g_initPoseReq
    # g_initPoseReq = f_req
    l_initPoints = []

    #Open initial pose file 
    with open(f_req.initPoseFile) as file:
        l_initPoints = yaml.load(file, Loader=yaml.FullLoader)
    
    rospy.loginfo("Initial points %s", l_initPoints)
    l_initPoint = l_initPoints[f_req.initPosId]

    #Set pose in rviz
    l_msg = PoseWithCovarianceStamped()
    l_msg.header.stamp = rospy.Time.now()
    l_msg.header.frame_id = "/map"

    print("Init point ", l_initPoint)

    l_msg.pose.pose.position.x = l_initPoint[0]
    l_msg.pose.pose.position.y = l_initPoint[1]

    l_msg.pose.pose.orientation.x = l_initPoint[2]
    l_msg.pose.pose.orientation.y = l_initPoint[3]
    l_msg.pose.pose.orientation.z = l_initPoint[4]
    l_msg.pose.pose.orientation.w = l_initPoint[5]

    #Obtain topic publisher
    rospy.loginfo("Setting initial pose to %s,%s", l_initPoint[0], l_initPoint[1])
    

    g_initPosePub.publish(l_msg)


def initialize():

    rospy.init_node('marsi_nav')

    global g_actionClient
    global g_initPosePub
    g_actionClient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    g_actionClient.wait_for_server()

    l_s1 = rospy.Service('marsi_nav_set_waypoints', setPointMapSrv, setWayPoints)
    l_s2 = rospy.Service('marsi_nav_move', moveSrv, move)

    rospy.Service('marsi_nav_set_initPose', setInitPoseSrv , setInitPose)
    g_initPosePub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10) 

    rospy.spin()

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        initialize()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
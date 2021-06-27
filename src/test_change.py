#! /usr/bin/env python3
import os

from matplotlib.pyplot import delaxes, xcorr
import rospy
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, PointStamped

import math
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

from Robot_lib import *
from Robot_paths_lib import *
from Robot_draw_lib import *
from Robot_sight_lib import *
from Robot_map_lib import *
from Robot_world_lib import *
from Robot_csv_lib import *
from Robot_goal_lib import *
from Program_config import *
from Robot_control_panel import *

config = Config()


def planner(robot_type=RobotType.circle,n):
    print(__file__ + " start!!")

    # set configuration of robot
    config.robot_type = robot_type
    robotvision = config.robot_vision
    
    # set same window size to capture pictures
    #plt.figure(figsize=(8,8))
    plt.figure(figsize=(7,7))
    
    # get user input
    runtimes = n
    mapname = menu_result.m
    worldname = menu_result.w

    start = np.array([menu_result.sx,menu_result.sy])
    goal = np.array([menu_result.gx,menu_result.gy])
    
    # current position of robot
    cpos = start
    
    # read world map
    if worldname is not None:
        read_map_from_world(worldname)
        ob = read_map_csv(worldname + ".csv")
    else:
        ob = read_map_csv(mapname)
        
    # traversal sight to draw visible visited places
    traversal_sight = []

    # active open points [global]
    ao_gobal = [] 

    r_goal = True
    s_goal = True

    no_way_togoal = False
    
    visible_graph = graph_intiailze()
    visited_path = []

    # for display information
    run_count = 0
    
    print ("\n____Robot is reaching to goal: {0} from start {1}".format(goal, start))
    
    while True:
        run_count += 1
        center = (cpos[0], cpos[1])
        
        print ("\n_____Run times:{0}, at {1}".format(run_count, center))
        
        # clean old data
        next_pt = []
        
        # scan to get sights at local
        closed_sights, open_sights = scan_around(center, robotvision, ob, goal)
        
        # check if the robot saw or reach the goal
        r_goal, s_goal = check_goal(center, goal, config, robotvision, closed_sights)
        
        if not s_goal and not r_goal:
            # get local open points
            open_local_pts = []
            if len(open_sights) > 0:
                open_sights = np.array(open_sights)
                open_local_pts = open_sights[:, 2]    # open_local_pts
                print ("open_local_pts,", open_local_pts)
                for i in range( len(open_local_pts)):
                    open_local_pts[i][0] = approximately_num(open_local_pts[i][0])
                    open_local_pts[i][1] = approximately_num(open_local_pts[i][1])

            # check whether open local points are active 
            if len(open_local_pts) : # new local found
                if len(traversal_sight) == 0:
                    # ranks new local open points
                    ao_local_pts = open_local_pts
                    ranks_new = np.array([ranking(center, pt, goal) for pt in open_local_pts])
                    # active open points at local
                    ao_local = np.concatenate((ao_local_pts, ranks_new), axis=1)
                    # add local to global
                    ao_gobal = np.array(ao_local)
                else:
                    open_local_pts_status = [inside_global_true_sight(pt, robotvision, traversal_sight) for pt in open_local_pts]
                    ao_local_pts = open_local_pts[np.logical_not(open_local_pts_status)]
                    print ("ao_local_pts,", ao_local_pts)
                    if len(ao_local_pts) > 0:
                        ranks_new = np.array([ranking(center, pt, goal) for pt in ao_local_pts])
                        # active open points at local
                        ao_local = np.concatenate((ao_local_pts, ranks_new), axis=1)
                        # add local to global
                        ao_gobal = np.concatenate((ao_gobal, ao_local), axis=0)
                    else:
                        ao_local_pts = []
                        #print ("No new open point at this local")
                
                graph_insert(visible_graph, center, ao_local_pts)
            
            else:   # there is no direction 
                print ("there is no direction reaching the goal")
                
            # pick next point to make a move
            picked_idx, next_pt = pick_next(ao_gobal)
            
            # find the shortest skeleton path from current position (center) to next point
            skeleton_path = BFS_skeleton_path(visible_graph, tuple(center), tuple(next_pt))

            # remove picked point from active global open point
            if picked_idx != -1:
                ao_gobal= np.delete(ao_gobal, picked_idx, axis=0)
            else:
                print ("No way to reach the goal!")
                no_way_togoal = True
        else:
            next_pt = goal
            # find the shortest path from center to next point
            skeleton_path = [center, goal]
            
        # record the path
        traversal_sight.append([center, closed_sights, open_sights])
        if print_traversal_sight:
            print ("traversal_sight:", traversal_sight)
            
        
        asp, critical_ls = approximately_shortest_path(skeleton_path, traversal_sight, robotvision)
        #asp = remove_validation(asp)
        visited_path.append(asp)
        
        #make a move from current position
        if not no_way_togoal:
            cpos = motion(cpos, next_pt)  # simulate robot  
        # Run n times for debugging
        if runtimes == run_count:
            break
        
        # check reaching goal
        if r_goal:
            print("Goal!!")
            break
        if no_way_togoal:
            break
    global dx 
    dx = next_pt[0] - cpos[0]
    global dy 
    dy = next_pt[1] - cpos[1]
    print ("visited_path:", visited_path)           
    print("Done")
    
    plt.show()
    return dx,dy

def callback(msg):
    #rospy.loginfo("x: %f y: %f" %(msg.pose.pose.position.x*100,msg.pose.pose.position.y*100))
    goal = PointStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "/map"
    goal.point.x = msg.point.x 
    goal.point.y = msg.point.y
    
    global goalx,goaly
    goalx = goal.point.x
    goaly = goal.point.y

    global cposx,cposy
    cposx = msg.pose.pose.position.x
    cposy = msg.pose.pose.position.y

    #rospy.loginfo("coordinates:x=%f y=%f" %(goal.point.x,goal.point.y))

class GoToPose():
    def __init__(self):

        self.goal_sent = False

        # What to do if shut down (e.g.Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)

        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        # Start moving
        self.move_base.send_goal(goal)

        # wait turtlebot3 60s to complete the task
        success = self.move_base.wait_for_result(rospy.Duration(60))

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            #reach the goal
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)


    
if __name__ == '__main__':
    try:
        rospy.init_node('nav_test')
        rospy.odom_sub = rospy.Subscriber('/odom', Odometry, callback)
        rospy.point_pub = rospy.Subscriber('/clicked_point', PointStamped, callback)
        rospy.spin()
        #save map to /home/tien/robot/cse_global_planner/src/map.pgm
        os.system('rosrun map_server map_saver -f /home/tien/robot/cse_global_planner/src/map.pgm')
        
        #position of the robot at first
        position = callback()
        cposx = position.msg.pose.pose.position.x
        cposy = position.msg.pose.pose.position.y
        
        #send goal to the robot
        goal = callback()
        goalx = goal.point.x
        goaly = goal.point.y

        #wait until robot reaches the goal
        while (cposx != goalx and cposy != goaly):
            
            #update position of the robot
            cposx = position.msg.pose.pose.position.x
            cposy = position.msg.pose.pose.position.y
            #save map for planning 
            os.system('rosrun map_server map_saver -f /home/tien/robot/cse_global_planner/src/map.pgm')

            #number of executing finding path
            n = 1
            start = np.array([cposx,cposy])
            goal = np.array([goalx,goaly])
            dx,dy = planner(robot_type=RobotType.circle,*goal,*start,*n)
            #update the next point to the robot in real world
            next_pointx = cposx + dx/20 #x=20 in simulation is equivalent to x=1 in real world 
            next_pointy = cposy + dy/20 #y=20 in simulation is equivalent to x=1 in real world 

            #robot moves to the next point
            navigator = GoToPose()
            position = {'x': next_pointx, 'y': next_pointy}
            quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': 0.000, 'r4': 1.000}    
            rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])

            #check if robot has moved to the next point
            success = navigator.goto(position, quaternion)
            if (success):
                rospy.loginfo("reach node")
                n+=1
            else:
                rospy.loginfo("failed to reach node")
            

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)
             

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")


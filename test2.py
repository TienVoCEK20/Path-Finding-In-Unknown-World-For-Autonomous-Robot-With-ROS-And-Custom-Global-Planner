'''
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target) 
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn
'''
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
print(__file__ + " start!!")

# set configuration of robot
config.robot_type = RobotType.circle
robotvision = config.robot_vision
    
# set same window size to capture pictures
#plt.figure(figsize=(8,8))
plt.figure(figsize=(7,7))
    
# get user input
menu_result = menu()
runtimes = 3
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

    
print ("\n____Robot is reaching to goal: {0} from start {1}".format(goal, start))
           

run_count = 0

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
                #print ("open_local_pts,", open_local_pts)
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
                    #print ("ao_local_pts,", ao_local_pts)
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
        #if print_traversal_sight:
            #print ("traversal_sight:", traversal_sight)
            
        
        asp, critical_ls = approximately_shortest_path(skeleton_path, traversal_sight, robotvision)
        #asp = remove_validation(asp)
        visited_path.append(asp)    
        dx = next_pt[0] - cpos[0]
        dy = next_pt[1] - cpos[1]
        #make a move from current position
        #if not no_way_togoal:
        cpos = motion(cpos, next_pt)  # simulate robot
        print("dx = %f, dy = %f" %(dx,dy))
        if run_count == runtimes:
            break    

# Autonomous Robot With ROS and Web Interface

![](robot.jpg)

#this is a project for doing a research path planning in an unknowing world that I am currently doing with many teachers in HCMUT

#Path planning in simulation

Terminal 1:
roscore

Terminal 2:
ssh ubuntu@{ip_address_of_remote_robot}

roslaunch turtlebot3_bringup turtlebot3_bringup.launch

Terminal 3:
roslaunch cse_global_planner cse-global-planner.launch

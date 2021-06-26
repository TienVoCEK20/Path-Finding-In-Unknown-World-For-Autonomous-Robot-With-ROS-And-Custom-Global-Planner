# Autonomous Robot With ROS and Web Interface

![](robot.jpg)

[Test SLAM video](https://drive.google.com/file/d/1RyLP1ap0MhS0q9NDTvzyzbdhAX0FgNUc/view?usp=sharing)

[Test Navigation video](https://drive.google.com/file/d/1xu4qLmRwJadYSF25RUl9RiRSYW9SUtGV/view?usp=sharing)

I am also implementing algorithm for global_planner. Will update later!!

Terminal 1:
roscore

Terminal 2:
ssh ubuntu@{ip_address_of_remote_robot}

roslaunch turtlebot3_bringup turtlebot3_bringup.launch

Terminal 3:
roslaunch cse_global_planner bringup.launch

Terminal 4:
rosrun cse_global_planner test.py

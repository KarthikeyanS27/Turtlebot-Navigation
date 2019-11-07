# Turtlebot-Navigation

This repository contains autonomous navigation research work conducted at the National University of Singapore (NUS) during Summer 2019. Work was developed in Python for ROS/Gazebo simulation environment. For complete written project report, see AutonomousNavigationBasedOnTurtleBot.pdf

This research project also produced a custom ROS package for four wheeled autonomous vehicles. For more information please see [ackermann_nav-ROS](https://github.com/apletta/ackermann_nav-ROS).

## Sample Usage
Main categories are:
- Autonomous Navigation to User Destination
- SLAM
- Multi-Robot SLAM
- Model Predictive Control Path and Speed Planning


### Autonomous Navigation to User Destination

User input determine which location robot will navigate to next. SLAM map must be created first. 
<img src="https://github.com/KarthikeyanS27/Turtlebot-Navigation/blob/master/Sample%20Pics/nav_route.png" alt="autonomous navigation" width="100%">

### SLAM
Experimentation with and evaluation of Google Cartographer.
<img src="https://github.com/KarthikeyanS27/Turtlebot-Navigation/blob/master/Sample%20Pics/cartographer.jpg" alt="google cartographer" width="100%">

Implementation of teleop control for mapping a simulation environment using gmapping SLAM.

<img src="https://github.com/KarthikeyanS27/Turtlebot-Navigation/blob/master/Sample%20Pics/teleop-slam.jpg" alt="teleop slam 1" width="40%"> <img src="https://github.com/KarthikeyanS27/Turtlebot-Navigation/blob/master/Sample%20Pics/world_city_slam.png" alt="teleop slam 2" width="40%">


### Multi-Robot SLAM

SLAM readings from multiple robots fused into a single map of the environment. 
<img src="https://github.com/KarthikeyanS27/Turtlebot-Navigation/blob/master/Sample%20Pics/multi-slam.png" alt="multi-slam" width="100%"> 


### Model Predictive Control Path and Speed Planning

Example of path planning and speed control for adaptive-horizon MPC. Utilizes on-line computation to dynamically avoid obstacles while navigating towards goal/checkpoint location.  
<img src="https://github.com/KarthikeyanS27/Turtlebot-Navigation/blob/master/Sample%20Pics/mpc.png" alt="on-line adaptive mpc" width="100%">


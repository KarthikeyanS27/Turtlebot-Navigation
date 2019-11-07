# Turtlebot-Navigation

This repository contains autonomous navigation research work conducted at the National University of Singapore (NUS) during Summer 2019. For complete written project report, see AutonomousNavigationBasedOnTurtleBot.pdf

Main categories are:
- Autonomous Navigation to User Destination
- SLAM
- Multi-Robot SLAM
- Model Predictive Control Path and Speed Planning

## Sample Usage
### Autonomous Navigation to User Destination

User input determine which location robot will navigate to next. SLAM map must be created first. 
<img src="https://github.com/KarthikeyanS27/Turtlebot-Navigation/blob/master/Sample%20Pics/nav_route2.png" alt="autonomous navigation" width="100%">

### SLAM
Experimentation with and evaluation of Google Cartographer
<img src="https://github.com/KarthikeyanS27/Turtlebot-Navigation/blob/master/Sample%20Pics/cartographer.png" alt="autonomous navigation" width="100%">

Implementation of teleop control for mapping a simulation environment using gmapping SLAM. 
<img src="https://github.com/KarthikeyanS27/Turtlebot-Navigation/blob/master/Sample%20Pics/nav_route2.png" alt="autonomous navigation" width="40%"> <img src="https://github.com/KarthikeyanS27/Turtlebot-Navigation/blob/master/Sample%20Pics/nav_route2.png" alt="autonomous navigation" width="40%">


### Multi-Robot SLAM

SLAM readings from multiple robots can be fused into a single map of the environment. 
<img src="https://github.com/KarthikeyanS27/Turtlebot-Navigation/blob/master/Sample%20Pics/cartographer.png" alt="autonomous navigation" width="100%">


### Model Predictive Control Path and Speed Planning

Example of path planning and speed control for adaptive MPC. On-line computation to dynamically avoid obstacles while navigation towards goal/checkpoint location.  
<img src="https://github.com/KarthikeyanS27/Turtlebot-Navigation/blob/master/Sample%20Pics/cartographer.png" alt="autonomous navigation" width="100%">


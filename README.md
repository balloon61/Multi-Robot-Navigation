# Multi-robot-navigation
## Desciption:
Multi robot navigation using tb2 because tb2 use stereo camera for the localization, it might lack of feature for localization therefore, you can select the ekf launch file instead of the origonal localization (AMCL).

## Requirement:
Turtlebot 2 install in your computer, gazebo, ros, octomap

### Installation of turtlebot2 
The setting of Turtlebot on Noetic is mostly follows this website:
https://gist.github.com/jeremyfix/0c5973aba508ee8b6e8d3c3077c6db1e




## Step:
```
# launch multi-robot in Gazebo and Rviz also run the navigation for every robot
roslaunch turtlebot_gazebo multi-robot.launch
# send goal to each robot
rosrun turtlebot_gazebo multi_pose_publisher.py
```

## Navigation demo
This demo send goal to each turtlebot which are (6.5, 5, 5), (5, 6.5, 0), (6.5, 6.5, 0) respectively.
### RViz
https://youtu.be/wRppShDb4Cg
### Gazebo
https://youtu.be/CdYUmaoeKLU

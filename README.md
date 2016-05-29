
# Autonomous Robots Final Project

This is a final project in Autonomous Robots class (VIBOT 2nd semester, University of Girona) done by Kaisar Kushibar(k.kushibar@gmail.com), Rodrigo Daudt (rcdaudt@gmail.com) and Songyou Peng (psy920710@gmail.com). 

### Demo
The demonstration of the algorithm can be seen [here].

### Requirements
* ROS Indigo
* Gazebo
* RViz
* Python 2.7

Create a catkin workspace and pull the code into the workspace src folder.

### How to Run
```sh
$ roslaunch ar_project gazebo.launch
$ roslaunch ar_project driver.launch x:=<x-coord> y:=<y-coord>
```
The first line launches the Gazebo simulator and RViz visualization tool. The arguments \<x-coord\>, \<y-coord\> are the goal point in pixels w.r.t world coordinate system.

Example (for the small 400*400 map)
```sh
$ roslaunch ar_project gazebo.launch
$ roslaunch ar_project driver.launch x:=200 y:=350
```



 [here]: <https://www.youtube.com/watch?v=5PaKWS52CIM>

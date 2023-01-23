## Coverage Drone
This repository contains source code for a simulation containing a drone for autonmous mapping of an area.

## Using this Repository

### Launch Files
- Bring drone in forests ( brings drone and world in Gazebo )
```
roslaunch coverage_drone drone_in_forest.launch
```
- Bring drone with Move_base integeration and Rviz with Gmapping 2D ( Drone ready to take Movebase Commands for Navigation )
```
roslaunch coverage_drone 2D_scan_with_movebase.launch
```
- Bring drone with Move_base integeration and Rviz with OctoMap 3D
```
roslaunch coverage_drone 3D_scan_with_movebase.launch
```
### Nodes
- Make the drone Hover (Fly drone to a specific height )
```
rosrun coverage_drone hover_drone.py
```
- Moving Drone with co-ordinates
```
rosrun coverage_drone move_base_goals.py
```
- Drive Drone using teleoperation keyboard ( manual Driving )
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
---

## Software Requirments
- Ubuntu 20.04
- ROS Noetic

- Install package dependencies
```
sudo apt-get install ros-noetic-move-base
sudo apt-get install ros-noetic-hector-gazebo-plugins
sudo apt-get install ros-noetic-navigation
sudo apt-get install ros-noetic-geographic-msgs
sudo apt-get install ros-noetic-map-server
sudo apt-get install ros-noetic-amcl
sudo apt-get install ros-noetic-slam-gmapping
sudo apt-get install ros-noetic-slam-toolbox
pip install scipy

```
---
## Leaving at
```
Problem with multi Robot Move Base Frame error
    - From map frame to Basefoot print
        - We have only one frame for MAP but base footprints are different
```
---
## Understanding Problem
- Turtlebot3 multi launch is having
    - Individual map and odom frame then comes base footprint
    - Link to explaining repo https://github.com/airuchen/multi_turtlebot3
    ```
    roslaunch turtlebot3_gazebo multi_turtlebot3_slam.launch ns:=tb3_0
    ```
- In our scenario we have
    - Single map and world frame.


- To write frames to opened directory in termianl
    ```
    rosrun tf2_tools view_frames.py
    ```
## ideas
- Run single drone with TF_PREFIX till slam navigation
    - ind out how to rename world to base footprint transform of ground_tf_publisher
## Resources
- https://github.com/RAFALAMAO/hector-quadrotor-noetic
- https://www.youtube.com/watch?v=-2IWfZjqoNc&ab_channel=RAFALAMAO
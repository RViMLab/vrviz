Visualization node for ROS using HTC Vive
=========================================

Prerequisites
-------------

This is a proof of concept level product for now, so I have not
kept careful track of what/how I installed to make this run. 

I started from the instructions [here](https://github.com/AndreGilerson/rviz_vive_plugin)
although I am using the most recent version of openvr (as of 2018-04-17).
So presumably most of the dependancies listed there are important.
The `openvr` repo is expected to be checked out in the same root as the `ctr_ros`
repo, for the linking to work.

Running with Steam Runtime
--------------------------

For now, this node requires being run as part of the steam runtime (or as shown in vrviz.launch):
```
rosrun --prefix '~/.steam/ubuntu12_32/steam-runtime/run.sh' vrviz vrviz_gl
```

Demonstration Launch Files
--------------------------

For a demo of showing a Turtlebot in Gazebo, install `ros-kinetic-turtlebot-gazebo` and run:
```
roslaunch vrviz turtlebot_demo.launch
```
This should load up the robot, and it can be controlled by pulling the trigger of the want and then moving/rotating the wand while the trigger is depressed. This launch file will fix the 'ground' of the vive to the `odom` frame.

For a demo showing a bagfile download the `demo_mapping.bag` file from [here](http://wiki.ros.org/rtabmap_ros) and run:
```
roslaunch vrviz point_cloud_demo.launch bagfile:=/path/to/demo_mapping.bag
```

Features
--------
 - The default RViz 1m grid
 - Scaling the VR world relative to the ROS world (currently fixed at startup)
 - Loading a robot model from the parameter server with `load_robot:=true`
 - Visualizing TF's (currently only TF's that have been referenced somewhere)
 - Visualizing PointCloud2 messages (currently expecting color)
 - Visualizing stereo pair image (currently expects one side-by-side image)

Limitations
-----------
 - The code is very much a work in progress, and many features are partially or incorrectly implemented.
 - The [SteamVR support for Ubuntu](https://github.com/ValveSoftware/SteamVR-for-Linux) is still in Beta, so be careful.
 - Running this code has been known to freeze computers for a few seconds or more, don't run with unsaved files open.



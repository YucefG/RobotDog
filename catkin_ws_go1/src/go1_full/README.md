# Go1-Full

Software package for Go1, using the Unitree SDK. This is a self-contained repository with correct versions to work with the current Go1 robots in the BioRob lab. New robots may have different SDKs. 

## Installation Notes

* Install ROS according to the appropriate [tutorial](http://wiki.ros.org/noetic/Installation/Ubuntu).
Make sure to source, add to your `.bashrc` file

* Create a catkin workspace according to the following [tutorial](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment). If you are using a BIOROB pc, this should be in `/data/<USER_NAME>/`, for example `/data/bellegar/`. If you want to make use of the BIOROB backups, you can put your files in your home directory somewhere, and symbolically link to your catkin workspace `src/` directory.  

* Install ROS dependencies, for noetic (also detailed in [unitree_ros](/unitree_ros)). If you are using a BIOROB PC and do not have access to sudo, this is probably already done for you.
```
sudo apt-get install ros-noetic-controller-interface  ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller
```

* Install dependencies for [unitree_legged_sdk](/unitree_legged_sdk), for example LCM. Again, if you are using a BIOROB PC, this is done for you. However, you still need to follow the directions to build. 

* Download Torch (explained below) and place it in `/data/`. If you are using a BIOROB PC, this is probably done for you. It is not yet used in the code, but this functionality may be added soon. 

* Compile with `catkin_make -DCMAKE_BUILD_TYPE=Release` and test launch and run_cpg as detailed below.


## Dependencies
The following repositories should be in your `catkin_ws/src` directory (the following are the ORIGINAL links, where the working versions are now included in this repo): 
* [unitree_ros](https://github.com/unitreerobotics/unitree_ros) 
* [unitree_ros_to_real](https://github.com/unitreerobotics/unitree_ros_to_real)
* [unitree_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk)

Avoid rewriting code for Gazebo/hardware - ideally only the exectuable should change. 

### Torch
Policies trained in Isaac Gym can be written out and tested both in Gazebo and on the hardware. 

* Make sure to download the cxx11 ABI version of libtorch:
https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.10.0%2Bcpu.zip

* Verify you can link properly by following this tutorial:
https://pytorch.org/tutorials/advanced/cpp_export.html

## Adding New Functionalities

The intended use is to create a directory in [go1_software](/go1_software) with a new "skill", such as "Balance", "MPC", "HRI" etc. Then, create executables in the same spirit as [run_cpg.cpp](/go1_software/src/exe/run_cpg.cpp) and [run_cpg_lcm.cpp](/go1_software/src/exe/run_cpg_lcm.cpp) to first test in Gazebo, and then on the hardware.


## Gazebo

### Run CPG Example

* `roslaunch unitree_gazebo normal.launch rname:=go1`
* `rosrun go1_software run_cpg`


## Hardware (ONLY ALLOWED WITH PERMISSION)

**For low-level control:** CPG with LCM, converting to ROS for data processing. **Make sure the robot is in "basic mode"** (after turning on, L2+A twice to lay down, L2+B to enter damping mode, then L1+L2+B. Now, can return to standing with L2+B, L2+A). **IMPORTANT**: see notes at the top of [run_cpg_lcm.cpp](/go1_software/src/exe/run_cpg_lcm.cpp) on what is allowed (i.e. no torque control, no large gains, nothing near robot power/torque limits, etc.).

* `roslaunch unitree_legged_real real.launch ctrl_level:=lowlevel` (optional)
* `rosrun go1_software run_cpg_lcm`

**For high-level control:** make sure the robot is in sport mode (i.e. walking mode, with START, NOT lying down). There is an example to use Unitree's high level API to control base orientation and moving forward/backward/left/right at [example_walk_lcm.cpp](/go1_software/src/exe/example_walk_lcm.cpp)

* `roslaunch unitree_legged_real real.launch ctrl_level:=highlevel` (optional)
* `rosrun go1_software example_walk_lcm`

## Notes

* Foot sensors are very inaccurate. Code attempts to find "zero" (in air) value, and then scales by gravity (may not be correct, but values are more reasonable). It is important to have a good estimate of these since the state estimation depends on this. 
* LCM data returned from robot seems to be a subset of total sensor info (i.e. compared to ROS msgs). 

## Resources

* [Unitree support youtube channel](https://www.youtube.com/channel/UCUyDbokbR3MhWo-GR1ansBQ?app=desktop)
* [Go1 Documentation from Unitree](https://drive.google.com/drive/u/2/folders/1QWYfP1b7W4IWBd1JxT-wUMzFepR2F7u9)


## TODO

* Integrate python interface.
* Clean up CMakeLists

# RobotDog
 Teaching a Robot New tricks - A project made by Guillaume Krafft and Yucef Grebici. Supervised by Guillaume Bellegarda and Auke Ijkspeert at BIOROB lab at EPFL. 

This Readme goes through the steps to set up the ROS environment to execute the final demonstration described in our report. 


# Installation steps
The following instructions are made to setup and test the ROS workspace,
and install the right libraries in the virtual environment.   
In the Hardware steps, it is assumed that all the prelimary have been done. 

Create a python virtual environment. Don't activate it yet.   
In each new terminal, run:  
```
source /opt/ros/noetic/setup.bash
```

## Go1 software
Go through the installation steps of [Guillaume Bellegarda's go1 repository](https://github.com/guillaume10/go1_full/tree/master). 

## Vision
#### Step 0
Run `roscore` in an independent terminal. 
#### Step 1: 
In a new terminal, activate the virtual environment. 
Also source the go1 workspace by running: 
```
source YOUR_WS_PATH/catkin_ws_go1/devel/setup.bash
```
Sourcing the go1 software enables us to use our custom message package, specially made to transmit the human keypoints through ROS topics. 
#### Step 2: 
Navigate to `Demos/ROS_demo_4`. 
Make sure the Intel RealSense camera is plugged on a **usb 3** port. Run and install the missing packages: 
```
python3 ros_video_topic.py
```
If an error concerning `Frame_info_3` is occuring, the catkin_ws_go1 package has not been properly sourced. A visualization window should appear if working.

Use `rostopic echo /ros_video_topic`, to check that the node is publishing, only when it sees a person in front of the camera.   
Keep it running.

#### Step 2: (Optionnal)
To gain time, adapt `video_setup.sh` to your file paths and only run it to launch the vision node. 

## Audio
For the audio part of the system, the 3 mics are mixed into one 3-channels virtual microphone input.

#### Step 1: Install PipeWire
You can install PipeWire instead of the default PulseAudio on Linux. You can follow the tutorial at this [link](https://linuxconfig.org/how-to-install-pipewire-on-ubuntu-linux) and restart the computer.

#### Step 2: 
Run start_pipewire

#### Step 3: 
Run createvirtualmic.sh if the sennhesier lavalier microphone is used of create createvirtualmic_wireless.sh if you use the labn's virtual microphone.

#### Debug step:
If there are issues with the virtual microphones you can run removevirtualmics.sh to remove the virtual microphones then repeat step 3 to recreate it.

#### Step 4:
Run search_audio_devices.py to list all the audio devices connected to the computer. Note the index of the 3 channels virtual microphone (it should be one of the last indexes) and modify AUDIO_DEVICE in the audio node of the demo you want to run.
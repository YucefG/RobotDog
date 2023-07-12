# RobotDog
 Teaching a Robot New tricks - A project made by Guillaume Krafft and Yucef Grebici. Supervised by Guillaume Bellegarda and Auke Ijkspeert at BIOROB lab at EPFL. 

This Readme goes through the steps to set up the ROS environment to execute the final demonstration, showed below. 



<p align="center">
  <img src="ezgif.com-optimize.gif" alt="Sublime's custom image", width="800"/>
</p>


# Installation
The following instructions are made to setup and test the ROS workspace,
and install the right libraries in the virtual environment.   
In the **Hardware Setup** steps, it is assumed that all the **Installation** steps have been achieved. 

Create a python virtual environment. Don't activate it yet.   
In each new terminal, run:  
```
source /opt/ros/noetic/setup.bash
```

## Go1 software
Go through the installation steps of [Guillaume Bellegarda's go1 repository](https://github.com/guillaume10/go1_full/tree/master), by placing the *go1_software* and the *custom_msg* packages from this repo into the *src* folder of your ROS workspace. Name this workspace *catkin_ws_go1*. 

## Vision
#### Step 0
Run `roscore` in an independent terminal. 
Install the [KeyWord Spotter folder](https://drive.google.com/drive/folders/1v-2Oh_Lg9upQyCzzcszk2qOSOBI7NTOM?usp=sharing) and place it in the *ROS_demo_5* folder
#### Step 1: 
In a new terminal, activate the virtual environment. 
Also source the go1 workspace by running: 
```
source YOUR_WS_PATH/catkin_ws_go1/devel/setup.bash
```
Sourcing the go1 software enables us to use our custom message package, specially made to transmit the human keypoints through ROS topics. 
#### Step 2: 
Navigate to *Demos/ROS_demo_4*. 
Make sure the Intel RealSense camera is plugged on a **usb 3** port. Run and install the missing packages: 
```
python3 ros_video_topic.py
```
If an error concerning *Frame_info_3* is occuring, the catkin_ws_go1 package has not been properly sourced. A visualization window should appear if working.

Use `rostopic echo /ros_video_topic`, to check that the node is publishing, only when it sees a person in front of the camera.   
Keep it running.

#### Step 2: (Optionnal)
To gain time, adapt *video_setup.sh* to your file paths and only run it to launch the vision node. 

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

#### Step 5: 
Navigate to *Demos/ROS_demo_4*. Activate the virtual environment.   
Run and install the missing packages: 
```
python3 ros_video_topic.py
```
If a too large offset is appearing (above 15), check that the microphones are working properly by listening to them in the computer settings. 

Use `rostopic echo /ros_audio_topic`, to check that the pronounced words are understood and well transmitted in the topic. 
Keep it running. 
## Brain
This part processes the information from the audio and camera nodes.

#### Step 0: 

In a new terminal, activate the virtual environment.   
Also source the go1 workspace by running: 
```
source YOUR_WS_PATH/catkin_ws_go1/devel/setup.bash
```

#### Step 1: 
Navigate to *Demos/ROS_demo_4*. 
Run and install the missing packages: 
```
python3 brain_HL.py
```
By listening to `cmd_mode`, you can check the robot-dog's logic.
Kill all terminals. 



# Hardware setup
Make sure that installation steps have been tested, and that each module works independently.

#### Step 1: Turn on the Unitree
Plug the ethernet cable to the robot. 
Turn on the Unitree, wait that it stands. 
Open a new terminal, export the computer's IP to ROS to bypass its aliases. Run roscore 
```
export ROS_MASTER_URI=http://128.178.148.56:11311
export ROS_IP=128.178.148.56
roscore
```

#### Step 2: Activate head's node 
1) ssh into the Unitree's computer  
`ssh unitree@192.168.123.14` password is ...   
2) Kill onboard camera processes (optional)  
`./start.bash`
3) Export ROS master to be the desktop (offboard PC from which we connect). This way, when you add ROS messages to communicate the status of the servos, everything is on the same network ([more info here](https://answers.ros.org/question/272065/specification-of-ros_master_uri-and-ros_hostname/)). 
```
export ROS_MASTER_URI=http://128.178.148.56:11311
export ROS_IP=128.178.148.56
```
4) Plug in the battery/USB and run the head node
python3 ~/DynamixelSDK/python/tests/protocol1_0/head_node.py

#### Step 3: Activate vision's node 
Run the steps starting from step 1, from installation in vision's part. 

#### Step 4: Activate audio's node 
Run the steps starting from step 5, from installation in audio's part. 

#### Step 5: Activate brain's node 
Open a new terminal, and run
```
export ROS_MASTER_URI=http://128.178.148.56:11311
export ROS_IP=128.178.148.56
```
Run all steps from installation in brain's part (except opening a new terminal).

At this stage, the head should track the human.

#### Step 6: Activate go1 hardware node
**Make sure you can control the robot with the adapted control pad.**
Source the go1 workspace and run the executable
```
source YOUR_WS_PATH/catkin_ws_go1/devel/setup.bash
rosrun go1_software controller_demo_4
```

The demonstration is ready to work! 

# Demos 2 and 3
To setup these demonstrations, the steps are merely the same.   
A few notes concerning them should be highlighted : 
- For each demo, we advise to create a new virtual environment, due to conflicting versions of libraries.   
- Camera visualization are not properly set up as on demonstration 4. 
- The head node is not used, unplug it. 
## Demo 2
Instead of `rosrun go1_software controller_demo_4` run `rosrun go1_software controller_demo_2`
## Demo 3
Instead of `rosrun go1_software controller_demo_4` run `rosrun go1_software controller_demo_3`

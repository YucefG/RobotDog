# RobotDog
 Teaching a Robot New tricks - A project made by Guillaume Krafft and Yucef Grebici. Supervised by Guillaume Bellegarda and Auke Ijkspeert at BIOROB lab at EPFL. 

# Audio
For the audio part of the system, the 3 mics are mixed into one 3-channels virtual microphone input.

# Step 1: Install PipeWire
You can install PipeWire instead of the default PulseAudio on Linux. You can follow the tutorial at this [link](https://linuxconfig.org/how-to-install-pipewire-on-ubuntu-linux) and restart the computer.

# Step 2: 
Run start_pipewire

# Step 3: 
Run createvirtualmic.sh if the sennhesier lavalier microphone is used of create createvirtualmic_wireless.sh if you use the labn's virtual microphone.

# Debug step:
If there are issues with the virtual microphones you can run removevirtualmics.sh to remove the virtual microphones then repeat step 3 to recreate it.

# Step 4:
Run search_audio_devices.py to list all the audio devices connected to the computer. Note the index of the 3 channels virtual microphone (it should be one of the last indexes) and modify AUDIO_DEVICE in the audio node of the demo you want to run.
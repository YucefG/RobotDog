[![kinetic-devel Status](https://github.com/ROBOTIS-GIT/DynamixelSDK/workflows/kinetic-devel/badge.svg)](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/kinetic-devel)
[![melodic-devel Status](https://github.com/ROBOTIS-GIT/DynamixelSDK/workflows/melodic-devel/badge.svg)](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/melodic-devel)
[![noetic-devel Status](https://github.com/ROBOTIS-GIT/DynamixelSDK/workflows/noetic-devel/badge.svg)](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/noetic-devel)
[![dashing-devel Status](https://github.com/ROBOTIS-GIT/DynamixelSDK/workflows/dashing-devel/badge.svg)](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/dashing-devel)
[![foxy-devel Status](https://github.com/ROBOTIS-GIT/DynamixelSDK/workflows/foxy-devel/badge.svg)](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/foxy-devel)
[![galactic-devel Status](https://github.com/ROBOTIS-GIT/DynamixelSDK/workflows/galactic-devel/badge.svg)](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/galactic-devel)
[![humble-devel Status](https://github.com/ROBOTIS-GIT/DynamixelSDK/workflows/humble-devel/badge.svg)](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/humble-devel)

<img src="http://emanual.robotis.com/assets/images/sw/sdk/dynamixel_sdk/overview/dynamixel_sdk_concept_logo.jpg">

## Dynamixel SDK
The ROBOTIS Dynamixel SDK is a software development kit that provides Dynamixel control functions using packet communication. The API is designed for Dynamixel actuators and Dynamixel-based platforms. For more information on Dynamixel SDK, please refer to the e-manual below.
- [ROBOTIS e-Manual for Dynamixel SDK](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)

## Supported Programming Languages
DynamixelSDK supports various programming languages.
- **C**: *Dynamic library and source code of this library and examples
- **C#** / **Java** / **MATLAB** / **LabVIEW**: Support based on dynamic library using C language
- **C++**: *Dynamic library and source code of this library and examples
- **Python**: Python module and examples
(* Dynamic library (*.dll, *.so, and *.dylib files) / .dll: dynamic-link library on Windows / .so: shared object on Linux / .dylib: dynamic library on MacOS)

For more information on ROS Packages for Dynamixel SDK, please refer to the ROS wiki pages below.
- http://wiki.ros.org/dynamixel_sdk
- http://wiki.ros.org/dynamixel_workbench
- http://wiki.ros.org/dynamixel_workbench_msgs

## Instructions (and differences from original repo)

- Check Python instructions [here](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/library_setup/python_linux/#python-linux)
- Python change from Ale: before doing the `setup.py` install you must change the `port_handler.py` file. The `writePort` function has been changed as follows:

```
def writePort(self, packet):
    ret = self.ser.write(packet)
    tmp = self.ser.timeout
    self.ser.timeout = None
    self.ser.read(ret)
    self.ser.timeout = tmp
    return ret
```

- For every stuff it sends, it immediately reads back the same amount of bytes, to remove them from the input buffer.
- The above has to be done BEFORE `python setup.py install`
- The motors we have are AX-12A (IDs 4 and 6), where `500` is the "nominal" value. 
- An example file of moving the head is [python/tests/protocol1_0/quadruped_head.py](python/tests/protocol1_0/quadruped_head.py)
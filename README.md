# ROS Drone Control 
follow the px4.io

# Steps:
1. Install the firmware (https://dev.px4.io/en/setup/dev_env_linux.html)
... Install the NuttX based firmware as the PX4 works on it.
... When installing toolchain, use the verison as per yout gcc version. Check it using:
```bash
gcc --verions
```
2. Now try to run the simulation to see if everything is working, the basic simulation may not start, so try to use gazebo simulator (https://dev.px4.io/en/simulation/gazebo.html)
...  more info on this is below.
3. Interface with ROS as per (https://dev.px4.io/en/simulation/ros_interface.html)
... You need to start mavros (can start it later as well)
... Then source the gazebo configs present in the Firmware downloaded. After that, use the lauch file specified to run the simulation environment.
4. Now you can start your node to communicate to mavros which inturn will communicate with px4 firmware. 

## if the error for protobuf occurs,
check if you have libprotobuf-dev (it is preinstalled with ubunutu)

if not,

then install it

and after that, install:
protobuf-compiler

## The Basic simultor (jamvsim) may not work correctly
use the gazebo simulator instead
```bash
make posix_sitl_default gazebo
```
and test it by entering
```bash
pxh> commander takeoff
```
and it should fly :p


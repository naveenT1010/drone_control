# ROS Drone Control 
follow the px4.io

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


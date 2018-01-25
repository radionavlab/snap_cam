# Snapdragon: Camera driver
This package provides tools to work with the Snapdragon Flight cameras. This package is a fork of the original [PX4/Snap_Cam](https://github.com/PX4/snap_cam) repository.

## Install Dependencies
Install the dependencies
```sh
sudo apt-get install libeigen3-dev sip-dev libyaml-cpp-dev libboost-dev cmake
```

To install OpenCV, [download](http://px4-tools.s3.amazonaws.com/opencv3_20160222-1_armhf.deb) and push the `.deb` package to the Snapdragon and install it using
```sh
dpkg -i opencv3_20160222-1_armhf.deb
```

ROS dependencies
```sh
sudo apt-get install ros-kinetic-mavlink ros-kinetic-tf ros-kinetic-orocos-toolchain ros-kinetic-angles ros-kinetic-tf2 ros-kinetic-tf2-ros
```

## Setup
### Create a catkin workspace
Create a catkin workspace (e.g. in /home/linaro)
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
```

### Clone repositories
```sh
cd ~/src
git clone https://github.com/ros-perception/vision_opencv
git clone https://github.com/ros-perception/image_common
git clone https://github.com/PX4/snap_cam.git
git clone https://github.com/radionavlab/gbx_ros_bridge_msgs
```

Initialize the Mavlink submodule:
```sh
cd ~/catkin_ws/src/snap_cam
git submodule update --init --recursive
```

Build:
```sh
cd ~/catkin_ws
catkin_make
```

## Run Snap Cam
```sh
sudo -s
roslaunch snap_cam main.launch
```

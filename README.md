# Overview

<!-- TODO: add overview -->

## Initialization

```bash
cd ~/Tasks
catkin_create_pkg noisy_turtle_playback
cd noisy_turtle_playback
git init
```

## Installing and setting up Turtlesim3

```bash
#Installing dependencies of turtlesim3
sudo apt-get update \
&& sudo apt-get install -y ros-noetic-joy ros-noetic-teleop-twist-joy \
ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
ros-noetic-rosserial-python ros-noetic-rosserial-client \
ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
#Installing turtlebot3 via Debian packages
sudo apt-get update \
  && sudo apt-get install -y ros-noetic-dynamixel-sdk \
  ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3
```

## Controlling turtlebot3 using ROS and arrow keys

Firstly, I didn't know the Keycodes for the arrow keys, \
but I knew that this command lets people control the `turtle1` in `turtlesim` using arrow keys: \
`rosrun turtlesim turtle_teleop_key`

So, I checked its [source code](https://github.com/ros/ros_tutorials/blob/noetic-devel/turtlesim/tutorials/teleop_turtle_key.cpp) and found out that the relevant keycodes are:

```cpp
#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_Q 0x71
```

Now, since the official `turtlebot3_teleop` package doesn't use the arrow keys, \
I cloned it: \
`git clone https://github.com/ROBOTIS-GIT/turtlebot3/` \
and created symlink to the package in the src folder of my workspace: \
`ln -s ~/git/turtlebot3/turtlebot3_teleop`

Then edited the `turtlebot3_teleop/nodes/turtlebot3_teleop_key` file, finally turned it into [this](https://gist.github.com/real-Sandip-Das/e97fc8cff416c464ccf0dcdcadc0c9cb) and saved it, \
rebuilt the package using `catkin build turtlebot3_teleop`

In order to test it, I installed the following package: \
`sudo apt install ros-noetic-turtlebot3-simulations`

Then, on one terminal I ran `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch` (after sourcing `devel/setup.bash`) \
and, in the other: `roslaunch turtlebot3_gazebo turtlebot3_world.launch` \
(on both, `export TURTLEBOT3_MODEL=waffle` needs to be done first)

It worked totally fine

![arrow-keys-driving](doc-images/arrow-keys.png)

## Recording topics into a bag file

Recorded all the topics that were being published using `rosbag record --all`

Kept recording while driving `turtlebot3` for quite a while, and pressed `Ctrl+C` to finish recording. \
The bag file ended up being 12.80 GB for some reason

## Adding Gaussian Noise to the velocity of `turtlebot3` and publishing it onto a new topic

# omni_ros2

We are stil trying to transplant moni_base_driver from ROS1 to ROS2. Therefore,  an omni_base_driver on ROS1 is needed currently. At this point, we are able to run lidar, SLAM, and navigation purely on ROS2.

## Installation

### ROS

Install Neuron-OmniBot base driver package.
```
source /opt/ros/melodic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/Adlink-ROS/Neuron-OmniBot.git
git checkout syy_omniBot  # choose specific robot model
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash

```

### ROS2

Create ros2_ws and download needed repos into src.
```
source /opt/ros/dashing/setup.bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
wget https://gist.githubusercontent.com/airuchen/6afb66a65947474405a8cbeb3aba7ecf/raw/60c32df7191200381d839d269ef68fccea994af1/omni_ros2.repos
vcs import src < omni_ros2.repos
```

Install dependencies.
```
rosdep install --from-paths src --ignore-src --default-yes
```

Build the packages.
```
colcon build --symlink-install
source ~/ros2_ws/install/local_setup.bash
```

## Usage

### Bring up omnibot

First, launch the base_driver on ROS1, which reads in IMU data, feedback odom, and allows us to give cmd_vel command.
```
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch omni_base_driver omni_base_driver_for_ros2.launch
```

In terminal 2, we open ros1_bridge to bridge all topics between ROS1 and ROS2.
```
source /opt/ros/melodic/setup.bash
source /opt/ros/dashing/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

Then in terminal 3, launch drive, which includes YDlidar, ekf, and robot_state_publisher.
```
ros2 launch omni_ros2 drive.launch.py
```

Now, we've successfully brought up the omnibot.

### SLAM

After bringing up the omnibot, we are able to use Cartographer package to map the environment. 
First, open Rviz.
```
ros2 launch omni_ros2 rviz.launch.py
```
Then, launch cartographer SLAM.
```
ros2 launch omni_ros2 slam.launch.py
```
Use keyboard to control the robot.
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Once finish mapping, save the map for nativation.
```
ros2 run nav2_map_server map_saver
```
Your map will be saved in home directory as a `map.pgm` and `map.yaml`.

### Navigation

Launch navigation2 and Rviz
```
ros2 launch omni_ros2 nav2.launch.py
ros2 launch omni_ros2 rviz.launch.py
```
In Rviz set Estimation, life_manager will continue to bring up cost map.
Use teleop to control omnibot allowing robot to localize its own location.
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Once amcl condence to certain location, set Goal in Rviz.


## Issue
1. Sometimes rviz could not read in map topic from map_server. Open rviz after nav2 will solve the issue.

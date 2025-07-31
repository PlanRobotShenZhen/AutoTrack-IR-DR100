
### 蓝海光电 LDS-50C-C30E 雷达

```bash
sudo ip addr add 192.168.158.200/24 dev <网卡名称>
sudo ip addr add 192.168.158.200/24 dev eth0
```

### IMU

```bash
sudo apt-get install libqt5serialport5-dev
# git submodule update --init --recursive
```

### gmapping

```bash
sudo apt install ros-${ROS_DISTRO}-gmapping
sudo apt install ros-${ROS_DISTRO}-map-server
```

```bash
# rosbag record /tf /tf_static /imu/data /scan
# rosbag play --clock *.bag
roslaunch bot_navigation gmapping.launch  use_ekf:=true
roslaunch bot_navigation save_map.launch
```

### navigation

```bash
sudo apt install ros-${ROS_DISTRO}-robot-localization
sudo apt install ros-${ROS_DISTRO}-robot-pose-ekf
sudo apt install ros-${ROS_DISTRO}-navigation
sudo apt install ros-${ROS_DISTRO}-navigation*
sudo apt install ros-${ROS_DISTRO}-move-base
sudo apt install ros-${ROS_DISTRO}-amcl
sudo apt install ros-${ROS_DISTRO}-dwa-local-planner
sudo apt install ros-${ROS_DISTRO}-teb-local-planner
```

```bash
roslaunch bot_navigation amcl.launch
roslaunch bot_navigation navigation.launch  use_ekf:=true  open_rviz:=true  slow_mode:=false  move_forward_only:=true 
```
## 许可证

本项目采用 [BSD 3-Clause] 许可证 - 查看 [LICENSE](LICENSE) 文件了解详情。本许可证适用于本项目的所有历史版本。

## 环境

- Ubuntu 18.04
- ROS Melodic

## 仿真环境导航

### 建图

启动仿真环境

```bash
roslaunch dr200_description gazebo.launch  publish_odom_tf:=false # 启动仿真环境
roslaunch bot_navigation gmapping.launch  use_ekf:=true  # gmapping建图
```

保存地图

```bash
roslaunch bot_navigation save_map.launch  # 保存地图
```

### 导航

然后将生成后的地图(*.pgm)替换到相应位置：`~/catkin_ws/src/bot_navigation/maps/`

可配置`~/catkin_ws/src/bot_navigation/param`目录下的导航规划器参数

具体可参考：[ROS::导航参数配置详解](https://blog.csdn.net/weixin_43928944/article/details/119571534)，`rosrun rqt_reconfigure rqt_reconfigure`可动态调整

```bash
roslaunch dr200_description gazebo.launch  publish_odom_tf:=false # 启动仿真环境
roslaunch bot_navigation navigation.launch  use_ekf:=true  open_rviz:=true  slow_mode:=false  move_forward_only:=true  # 导航
```

## 真实环境导航

**确保已连接以下模块：**

- IMU
- LDS-50C-C30E
- DR200 小车底盘

### 建图

启动真实环境下的小车驱动与描述，并录制bag

```bash
roslaunch dr200_description run.launch odom_publish_tf:=false   # 启动真实环境下的小车驱动与描述
roslaunch bot_navigation gmapping.launch  # gmapping建图
```

保存地图

```bash
roslaunch bot_navigation save_map.launch  # 保存地图
```

导航

```bash
roslaunch dr200_description run.launch odom_publish_tf:=false   # 启动真实环境下的小车驱动与描述
roslaunch bot_navigation navigation.launch  use_ekf:=true  open_rviz:=true  slow_mode:=false  move_forward_only:=true  # 导航
```

# 桌面快捷方式

## xrandr

```bash
sudo chmod +x Resolution.desktop
cp Resolution.desktop ~/Desktop/
```
and click `Resolution.desktop` on desktop.

## Start

```bash
sudo chmod +x start.sh 
sudo chmod +x start_autotrack.desktop
cp start_autotrack.desktop ~/Desktop/
```

and click `start_autotrack.desktop` on desktop.

## 常用命令

  - 清除目标点: `rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}`

  - qt 查看 tf_tree: `rosrun rqt_tf_tree rqt_tf_tree`

  - 生成tf_tree的frames.gv和frames.pdf: `rosrun tf view_frames`

  - git浅拷贝: `git clone --depth 1 --single-branch --branch <branch_name> <url>`

## FAQ

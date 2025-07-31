#!/usr/bin/env python
# coding=utf-8
"""
地图保存脚本, 支持自动边界修剪功能

使用方法: 
1. 默认保存并修剪边界: roslaunch bot_navigation save_map.launch
2. 保存但不修剪边界: roslaunch bot_navigation save_map.launch trim_boundaries:=false

修剪功能: 自动检测有效内容区域, 移除空白边界, 保留10像素边距
"""

import os
import datetime
import rospy
import rospkg
import yaml
import numpy as np
import cv2
from subprocess import call

map_name = "map"

def trim_map_boundaries(pgm_path, yaml_path):
    """修剪地图边界, 移除空白区域"""
    # 读取地图图像
    img_array = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if img_array is None:
        rospy.logerr("无法读取地图文件: {}".format(pgm_path))
        return
    
    # 找到非空白区域 (205是未知区域的灰度值)
    non_empty_mask = img_array != 205
    if not np.any(non_empty_mask):
        rospy.logwarn("地图完全为空, 跳过边界修剪")
        return
    
    # 获取边界索引
    rows = np.any(non_empty_mask, axis=1)
    cols = np.any(non_empty_mask, axis=0)
    row_min, row_max = np.where(rows)[0][[0, -1]]
    col_min, col_max = np.where(cols)[0][[0, -1]]
    
    # 添加边距
    margin = 10
    row_min = max(0, row_min - margin)
    row_max = min(img_array.shape[0] - 1, row_max + margin)
    col_min = max(0, col_min - margin)
    col_max = min(img_array.shape[1] - 1, col_max + margin)
    
    # 裁剪并保存图像
    cropped_array = img_array[row_min:row_max+1, col_min:col_max+1]
    trimmed_pgm_path = pgm_path.replace('.pgm', '_trimmed.pgm')
    cv2.imwrite(trimmed_pgm_path, cropped_array)
    
    # 更新YAML配置
    with open(yaml_path, 'r') as f:
        yaml_data = yaml.safe_load(f)
    
    resolution = yaml_data['resolution']
    old_origin = yaml_data['origin']
    
    # 计算新原点
    new_origin_x = float(old_origin[0] + col_min * resolution)
    new_origin_y = float(old_origin[1] + (img_array.shape[0] - row_max - 1) * resolution)
    
    yaml_data['image'] = "{}_trimmed.pgm".format(map_name)
    yaml_data['origin'] = [new_origin_x, new_origin_y, float(old_origin[2])]
    
    # 保存新YAML文件
    trimmed_yaml_path = yaml_path.replace('.yaml', '_trimmed.yaml')
    with open(trimmed_yaml_path, 'w') as f:
        yaml.dump(yaml_data, f, default_flow_style=False, allow_unicode=True)
    
    rospy.loginfo("地图修剪完成: {}x{} -> {}x{}".format(
        img_array.shape[1], img_array.shape[0], 
        cropped_array.shape[1], cropped_array.shape[0]))

def save_map():
    # 获取参数
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    rospack = rospkg.RosPack()
    maps_dir = rospy.get_param("~maps_dir", os.path.join(rospack.get_path('bot_navigation'), 'maps'))
    target_dir = os.path.join(maps_dir, timestamp)
    trim_boundaries = rospy.get_param("~trim_boundaries", True)
    
    # 创建目录并保存地图
    if not os.path.exists(target_dir):
        os.makedirs(target_dir)
    rospy.loginfo("保存地图到: {}".format(target_dir))
    
    # 切换目录保存地图（生成相对路径）
    original_cwd = os.getcwd()
    try:
        os.chdir(target_dir)
        call(["rosrun", "map_server", "map_saver", "-f", map_name])
    finally:
        os.chdir(original_cwd)
    
    # 修剪边界
    if trim_boundaries:
        pgm_path = os.path.join(target_dir, "{}.pgm".format(map_name))
        yaml_path = os.path.join(target_dir, "{}.yaml".format(map_name))
        if os.path.exists(pgm_path) and os.path.exists(yaml_path):
            trim_map_boundaries(pgm_path, yaml_path)
        else:
            rospy.logerr("地图文件未找到")
    else:
        rospy.loginfo("跳过边界修剪")

if __name__ == "__main__":
    rospy.init_node("save_map_with_timestamp")
    save_map()

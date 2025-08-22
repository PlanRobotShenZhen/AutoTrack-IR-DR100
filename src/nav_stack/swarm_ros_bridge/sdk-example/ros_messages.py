#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模拟ROS消息类, 用于ZeroMQ通信
"""

import struct
import time
from io import BytesIO


class Time:
    """ROS Time消息类"""
    def __init__(self, secs=0, nsecs=0):
        self.secs = secs
        self.nsecs = nsecs
    
    @classmethod
    def now(cls):
        """获取当前时间"""
        current_time = time.time()
        secs = int(current_time)
        nsecs = int((current_time - secs) * 1e9)
        return cls(secs, nsecs)
    
    def serialize(self, buff):
        """序列化Time"""
        buff.write(struct.pack('<I', self.secs))
        buff.write(struct.pack('<I', self.nsecs))
    
    def deserialize(self, buff):
        """反序列化Time"""
        self.secs = struct.unpack('<I', buff.read(4))[0]
        self.nsecs = struct.unpack('<I', buff.read(4))[0]


class Header:
    """ROS Header消息类"""
    def __init__(self, seq=0, stamp=None, frame_id=""):
        self.seq = seq
        self.stamp = stamp if stamp else Time()
        self.frame_id = frame_id
    
    def serialize(self, buff):
        """序列化Header"""
        buff.write(struct.pack('<I', self.seq))
        self.stamp.serialize(buff)
        frame_id_bytes = self.frame_id.encode('utf-8')
        buff.write(struct.pack('<I', len(frame_id_bytes)))
        buff.write(frame_id_bytes)
    
    def deserialize(self, buff):
        """反序列化Header"""
        self.seq = struct.unpack('<I', buff.read(4))[0]
        self.stamp.deserialize(buff)
        frame_id_len = struct.unpack('<I', buff.read(4))[0]
        self.frame_id = buff.read(frame_id_len).decode('utf-8')


class Point:
    """ROS Point消息类"""
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z
    
    def serialize(self, buff):
        """序列化Point"""
        buff.write(struct.pack('<d', self.x))
        buff.write(struct.pack('<d', self.y))
        buff.write(struct.pack('<d', self.z))
    
    def deserialize(self, buff):
        """反序列化Point"""
        self.x = struct.unpack('<d', buff.read(8))[0]
        self.y = struct.unpack('<d', buff.read(8))[0]
        self.z = struct.unpack('<d', buff.read(8))[0]


class Quaternion:
    """ROS Quaternion消息类"""
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
    
    def serialize(self, buff):
        """序列化Quaternion"""
        buff.write(struct.pack('<d', self.x))
        buff.write(struct.pack('<d', self.y))
        buff.write(struct.pack('<d', self.z))
        buff.write(struct.pack('<d', self.w))
    
    def deserialize(self, buff):
        """反序列化Quaternion"""
        self.x = struct.unpack('<d', buff.read(8))[0]
        self.y = struct.unpack('<d', buff.read(8))[0]
        self.z = struct.unpack('<d', buff.read(8))[0]
        self.w = struct.unpack('<d', buff.read(8))[0]


class Pose:
    """ROS Pose消息类"""
    def __init__(self, position=None, orientation=None):
        self.position = position if position else Point()
        self.orientation = orientation if orientation else Quaternion()
    
    def serialize(self, buff):
        """序列化Pose"""
        self.position.serialize(buff)
        self.orientation.serialize(buff)
    
    def deserialize(self, buff):
        """反序列化Pose"""
        self.position.deserialize(buff)
        self.orientation.deserialize(buff)


class PoseStamped:
    """ROS PoseStamped消息类"""
    def __init__(self, header=None, pose=None):
        self.header = header if header else Header()
        self.pose = pose if pose else Pose()
    
    def serialize(self, buff):
        """序列化PoseStamped到字节流"""
        self.header.serialize(buff)
        self.pose.serialize(buff)
    
    def deserialize(self, buff):
        """从字节流反序列化PoseStamped"""
        self.header.deserialize(buff)
        self.pose.deserialize(buff)
    
    def to_bytes(self):
        """转换为字节数据"""
        buff = BytesIO()
        self.serialize(buff)
        data = buff.getvalue()
        buff.close()
        return data
    
    @classmethod
    def from_bytes(cls, data):
        """从字节数据创建PoseStamped"""
        msg = cls()
        buff = BytesIO(data)
        msg.deserialize(buff)
        buff.close()
        return msg
    
    def __str__(self):
        """字符串表示"""
        return f"""PoseStamped:
  header:
    seq: {self.header.seq}
    stamp: {self.header.stamp.secs}.{self.header.stamp.nsecs:09d}
    frame_id: '{self.header.frame_id}'
  pose:
    position:
      x: {self.pose.position.x}
      y: {self.pose.position.y}
      z: {self.pose.position.z}
    orientation:
      x: {self.pose.orientation.x}
      y: {self.pose.orientation.y}
      z: {self.pose.orientation.z}
      w: {self.pose.orientation.w}"""


class Vector3:
    """ROS Vector3消息类"""
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z
    
    def serialize(self, buff):
        """序列化Vector3"""
        buff.write(struct.pack('<d', self.x))
        buff.write(struct.pack('<d', self.y))
        buff.write(struct.pack('<d', self.z))
    
    def deserialize(self, buff):
        """反序列化Vector3"""
        self.x = struct.unpack('<d', buff.read(8))[0]
        self.y = struct.unpack('<d', buff.read(8))[0]
        self.z = struct.unpack('<d', buff.read(8))[0]


class Twist:
    """ROS Twist消息类"""
    def __init__(self, linear=None, angular=None):
        self.linear = linear if linear else Vector3()
        self.angular = angular if angular else Vector3()

    def serialize(self, buff):
        """序列化Twist到字节流"""
        self.linear.serialize(buff)
        self.angular.serialize(buff)

    def deserialize(self, buff):
        """从字节流反序列化Twist"""
        self.linear.deserialize(buff)
        self.angular.deserialize(buff)

    def to_bytes(self):
        """转换为字节数据"""
        buff = BytesIO()
        self.serialize(buff)
        data = buff.getvalue()
        buff.close()
        return data

    @classmethod
    def from_bytes(cls, data):
        """从字节数据创建Twist"""
        msg = cls()
        buff = BytesIO(data)
        msg.deserialize(buff)
        buff.close()
        return msg

    def __str__(self):
        """字符串表示"""
        return f"""Twist:
  linear:
    x: {self.linear.x}
    y: {self.linear.y}
    z: {self.linear.z}
  angular:
    x: {self.angular.x}
    y: {self.angular.y}
    z: {self.angular.z}"""


class PoseWithCovariance:
    """ROS PoseWithCovariance消息类"""
    def __init__(self, pose=None, covariance=None):
        self.pose = pose if pose else Pose()
        self.covariance = covariance if covariance else [0.0] * 36  # 6x6矩阵

    def serialize(self, buff):
        """序列化PoseWithCovariance"""
        self.pose.serialize(buff)
        for val in self.covariance:
            buff.write(struct.pack('<d', val))

    def deserialize(self, buff):
        """反序列化PoseWithCovariance"""
        self.pose.deserialize(buff)
        self.covariance = []
        for _ in range(36):
            self.covariance.append(struct.unpack('<d', buff.read(8))[0])


class PoseWithCovarianceStamped:
    """ROS PoseWithCovarianceStamped消息类"""
    def __init__(self, header=None, pose=None):
        self.header = header if header else Header()
        self.pose = pose if pose else PoseWithCovariance()

    def serialize(self, buff):
        """序列化PoseWithCovarianceStamped到字节流"""
        self.header.serialize(buff)
        self.pose.serialize(buff)

    def deserialize(self, buff):
        """从字节流反序列化PoseWithCovarianceStamped"""
        self.header.deserialize(buff)
        self.pose.deserialize(buff)

    def to_bytes(self):
        """转换为字节数据"""
        buff = BytesIO()
        self.serialize(buff)
        data = buff.getvalue()
        buff.close()
        return data

    @classmethod
    def from_bytes(cls, data):
        """从字节数据创建PoseWithCovarianceStamped"""
        msg = cls()
        buff = BytesIO(data)
        msg.deserialize(buff)
        buff.close()
        return msg


class GoalID:
    """ROS GoalID消息类"""
    def __init__(self, stamp=None, id=""):
        self.stamp = stamp if stamp else Time()
        self.id = id

    def serialize(self, buff):
        """序列化GoalID"""
        self.stamp.serialize(buff)
        id_bytes = self.id.encode('utf-8')
        buff.write(struct.pack('<I', len(id_bytes)))
        buff.write(id_bytes)

    def deserialize(self, buff):
        """反序列化GoalID"""
        self.stamp.deserialize(buff)
        id_len = struct.unpack('<I', buff.read(4))[0]
        self.id = buff.read(id_len).decode('utf-8')


class GoalStatus:
    """ROS GoalStatus消息类"""
    # 状态常量
    PENDING = 0
    ACTIVE = 1
    PREEMPTED = 2
    SUCCEEDED = 3
    ABORTED = 4
    REJECTED = 5
    PREEMPTING = 6
    RECALLING = 7
    RECALLED = 8
    LOST = 9

    def __init__(self, goal_id=None, status=0, text=""):
        self.goal_id = goal_id if goal_id else GoalID()
        self.status = status
        self.text = text

    def serialize(self, buff):
        """序列化GoalStatus"""
        self.goal_id.serialize(buff)
        buff.write(struct.pack('<B', self.status))
        text_bytes = self.text.encode('utf-8')
        buff.write(struct.pack('<I', len(text_bytes)))
        buff.write(text_bytes)

    def deserialize(self, buff):
        """反序列化GoalStatus"""
        self.goal_id.deserialize(buff)
        self.status = struct.unpack('<B', buff.read(1))[0]
        text_len = struct.unpack('<I', buff.read(4))[0]
        self.text = buff.read(text_len).decode('utf-8')


class GoalStatusArray:
    """ROS GoalStatusArray消息类"""
    def __init__(self, header=None, status_list=None):
        self.header = header if header else Header()
        self.status_list = status_list if status_list else []

    def serialize(self, buff):
        """序列化GoalStatusArray到字节流"""
        self.header.serialize(buff)
        buff.write(struct.pack('<I', len(self.status_list)))
        for status in self.status_list:
            status.serialize(buff)

    def deserialize(self, buff):
        """从字节流反序列化GoalStatusArray"""
        self.header.deserialize(buff)
        list_len = struct.unpack('<I', buff.read(4))[0]
        self.status_list = []
        for _ in range(list_len):
            status = GoalStatus()
            status.deserialize(buff)
            self.status_list.append(status)

    def to_bytes(self):
        """转换为字节数据"""
        buff = BytesIO()
        self.serialize(buff)
        data = buff.getvalue()
        buff.close()
        return data

    @classmethod
    def from_bytes(cls, data):
        """从字节数据创建GoalStatusArray"""
        msg = cls()
        buff = BytesIO(data)
        msg.deserialize(buff)
        buff.close()
        return msg


class MapMetaData:
    """ROS MapMetaData消息类"""
    def __init__(self, map_load_time=None, resolution=0.0, width=0, height=0, origin=None):
        self.map_load_time = map_load_time if map_load_time else Time()
        self.resolution = resolution
        self.width = width
        self.height = height
        self.origin = origin if origin else Pose()

    def serialize(self, buff):
        """序列化MapMetaData"""
        self.map_load_time.serialize(buff)
        buff.write(struct.pack('<f', self.resolution))
        buff.write(struct.pack('<I', self.width))
        buff.write(struct.pack('<I', self.height))
        self.origin.serialize(buff)

    def deserialize(self, buff):
        """反序列化MapMetaData"""
        self.map_load_time.deserialize(buff)
        self.resolution = struct.unpack('<f', buff.read(4))[0]
        self.width = struct.unpack('<I', buff.read(4))[0]
        self.height = struct.unpack('<I', buff.read(4))[0]
        self.origin.deserialize(buff)


class OccupancyGrid:
    """ROS OccupancyGrid消息类"""
    def __init__(self, header=None, info=None, data=None):
        self.header = header if header else Header()
        self.info = info if info else MapMetaData()
        self.data = data if data else []

    def serialize(self, buff):
        """序列化OccupancyGrid到字节流"""
        self.header.serialize(buff)
        self.info.serialize(buff)
        buff.write(struct.pack('<I', len(self.data)))
        for val in self.data:
            buff.write(struct.pack('<b', val))  # int8

    def deserialize(self, buff):
        """从字节流反序列化OccupancyGrid"""
        self.header.deserialize(buff)
        self.info.deserialize(buff)
        data_len = struct.unpack('<I', buff.read(4))[0]
        self.data = []
        for _ in range(data_len):
            self.data.append(struct.unpack('<b', buff.read(1))[0])

    def to_bytes(self):
        """转换为字节数据"""
        buff = BytesIO()
        self.serialize(buff)
        data = buff.getvalue()
        buff.close()
        return data

    @classmethod
    def from_bytes(cls, data):
        """从字节数据创建OccupancyGrid"""
        msg = cls()
        buff = BytesIO(data)
        msg.deserialize(buff)
        buff.close()
        return msg


class MoveBaseGoal:
    """ROS MoveBaseGoal消息类"""
    def __init__(self, target_pose=None):
        self.target_pose = target_pose if target_pose else PoseStamped()

    def serialize(self, buff):
        """序列化MoveBaseGoal"""
        self.target_pose.serialize(buff)

    def deserialize(self, buff):
        """反序列化MoveBaseGoal"""
        self.target_pose.deserialize(buff)


class MoveBaseActionGoal:
    """ROS MoveBaseActionGoal消息类"""
    def __init__(self, header=None, goal_id=None, goal=None):
        self.header = header if header else Header()
        self.goal_id = goal_id if goal_id else GoalID()
        self.goal = goal if goal else MoveBaseGoal()

    def serialize(self, buff):
        """序列化MoveBaseActionGoal到字节流"""
        self.header.serialize(buff)
        self.goal_id.serialize(buff)
        self.goal.serialize(buff)

    def deserialize(self, buff):
        """从字节流反序列化MoveBaseActionGoal"""
        self.header.deserialize(buff)
        self.goal_id.deserialize(buff)
        self.goal.deserialize(buff)

    def to_bytes(self):
        """转换为字节数据"""
        buff = BytesIO()
        self.serialize(buff)
        data = buff.getvalue()
        buff.close()
        return data

    @classmethod
    def from_bytes(cls, data):
        """从字节数据创建MoveBaseActionGoal"""
        msg = cls()
        buff = BytesIO(data)
        msg.deserialize(buff)
        buff.close()
        return msg

    def __str__(self):
        """字符串表示"""
        return f"""MoveBaseActionGoal:
  header:
    seq: {self.header.seq}
    stamp: {self.header.stamp.secs}.{self.header.stamp.nsecs:09d}
    frame_id: '{self.header.frame_id}'
  goal_id:
    stamp: {self.goal_id.stamp.secs}.{self.goal_id.stamp.nsecs:09d}
    id: '{self.goal_id.id}'
  goal:
    target_pose:
      header:
        seq: {self.goal.target_pose.header.seq}
        stamp: {self.goal.target_pose.header.stamp.secs}.{self.goal.target_pose.header.stamp.nsecs:09d}
        frame_id: '{self.goal.target_pose.header.frame_id}'
      pose:
        position:
          x: {self.goal.target_pose.pose.position.x}
          y: {self.goal.target_pose.pose.position.y}
          z: {self.goal.target_pose.pose.position.z}
        orientation:
          x: {self.goal.target_pose.pose.orientation.x}
          y: {self.goal.target_pose.pose.orientation.y}
          z: {self.goal.target_pose.pose.orientation.z}
          w: {self.goal.target_pose.pose.orientation.w}"""


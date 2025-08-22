#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ZeroMQ调度中心模拟器
用于测试robot_communication_local_test.yaml配置的ZeroMQ通讯功能
使用独立的ROS消息类，不依赖ROS包
"""

import zmq
import time
import threading
import struct
from ros_messages import (
    PoseStamped, Twist, Header, Time, Point, Quaternion, Vector3,
    PoseWithCovarianceStamped, PoseWithCovariance, GoalStatusArray,
    GoalStatus, GoalID, OccupancyGrid, MapMetaData, MoveBaseGoal
)

class DispatchCenterSimulator:
    def __init__(self):
        self.context = zmq.Context()

        # 发送socket (PUB模式) - 调度中心发送给机器人
        self.goal_sender = self.context.socket(zmq.PUB)
        self.goal_sender.bind("tcp://*:4001")  # /move_base_simple/goal

        self.cmd_vel_sender = self.context.socket(zmq.PUB)
        self.cmd_vel_sender.bind("tcp://*:4002")  # /cmd_vel

        self.cancel_sender = self.context.socket(zmq.PUB)
        self.cancel_sender.bind("tcp://*:4003")  # /move_base/cancel

        # 接收socket (SUB模式) - 调度中心接收机器人数据
        self.map_receiver = self.context.socket(zmq.SUB)
        self.map_receiver.connect("tcp://127.0.0.1:3001")  # /map
        self.map_receiver.setsockopt(zmq.SUBSCRIBE, b"")

        self.amcl_receiver = self.context.socket(zmq.SUB)
        self.amcl_receiver.connect("tcp://127.0.0.1:3002")  # /amcl_pose
        self.amcl_receiver.setsockopt(zmq.SUBSCRIBE, b"")

        self.goal_receiver = self.context.socket(zmq.SUB)
        self.goal_receiver.connect("tcp://127.0.0.1:3003")  # /move_base/current_goal
        self.goal_receiver.setsockopt(zmq.SUBSCRIBE, b"")

        self.status_receiver = self.context.socket(zmq.SUB)
        self.status_receiver.connect("tcp://127.0.0.1:3004")  # /move_base/status
        self.status_receiver.setsockopt(zmq.SUBSCRIBE, b"")

        self.result_receiver = self.context.socket(zmq.SUB)
        self.result_receiver.connect("tcp://127.0.0.1:3005")  # /move_base/result
        self.result_receiver.setsockopt(zmq.SUBSCRIBE, b"")

        # 接收计数器
        self.recv_counts = {
            'map': 0,
            'amcl_pose': 0,
            'current_goal': 0,
            'nav_status': 0,
            'nav_result': 0
        }

        self.running = True
        print("调度中心模拟器启动")
        time.sleep(1)

    def create_pose_stamped_data(self, x=3.0, y=-1.5, z=0.0, qx=0.0, qy=0.0, qz=-0.7, qw=0.7):
        """创建PoseStamped消息的正确二进制数据"""
        # 使用独立的PoseStamped类
        pose_msg = PoseStamped()

        # 设置Header
        pose_msg.header.seq = 0
        pose_msg.header.stamp = Time(secs=0, nsecs=0)
        pose_msg.header.frame_id = "map"

        # 设置Pose
        pose_msg.pose.position = Point(x=x, y=y, z=z)
        pose_msg.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        # 序列化为字节数据
        return pose_msg.to_bytes()

    def create_twist_data(self, linear_x=0.0, linear_y=0.0, linear_z=0.0,
                         angular_x=0.0, angular_y=0.0, angular_z=0.0):
        """创建Twist消息的正确二进制数据"""
        # 使用独立的Twist类
        twist_msg = Twist()
        twist_msg.linear = Vector3(x=linear_x, y=linear_y, z=linear_z)
        twist_msg.angular = Vector3(x=angular_x, y=angular_y, z=angular_z)

        # 序列化为字节数据
        return twist_msg.to_bytes()

    def send_zmq_message(self, socket, binary_data):
        """发送ZeroMQ消息，与bridge_node格式兼容"""
        try:
            # 获取数据长度
            data_len = len(binary_data)

            # 使用多部分消息格式，与zmqpp兼容
            # 第一部分：data_len (大端序，与zmqpp兼容)
            # 第二部分：二进制数据
            socket.send_multipart([
                struct.pack('>Q', data_len),  # 大端序，8字节无符号整数
                binary_data                   # 二进制数据
            ])
            return True
        except Exception as e:
            print(f"发送ZeroMQ消息失败: {e}")
            return False

    def parse_zmq_message(self, raw_message, msg_type="unknown"):
        """解析ZeroMQ消息并反序列化ROS消息"""
        try:
            # 检查是否是多部分消息
            if isinstance(raw_message, list):
                # 多部分消息格式：[data_length, serialized_data]
                if len(raw_message) >= 2:
                    # bridge_node发送的data_length是大端序
                    data_length = struct.unpack('>Q', raw_message[0])[0]
                    serialized_data = raw_message[1]
                else:
                    return f"多部分消息格式错误: 部分数={len(raw_message)}"
            else:
                # 单一消息格式：[data_length][serialized_data]
                if len(raw_message) < 8:
                    return f"消息太短: {len(raw_message)} 字节"

                # bridge_node发送的data_length是大端序
                try:
                    data_length = struct.unpack('>Q', raw_message[:8])[0]
                except Exception as e:
                    return f"无法解析数据长度: {e}"

                # 检查数据长度是否合理
                if data_length > len(raw_message) or data_length > 10*1024*1024:  # 10MB限制
                    return f"数据长度异常: {data_length}, 消息长度: {len(raw_message)}"

                serialized_data = raw_message[8:8+data_length]

            # 验证序列化数据长度
            if len(serialized_data) != data_length:
                return f"序列化数据长度不匹配: 期望{data_length}, 实际{len(serialized_data)}"

            # 根据消息类型反序列化
            if msg_type == "amcl_pose":
                try:
                    msg = PoseWithCovarianceStamped.from_bytes(serialized_data)
                    return f"x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}"
                except Exception as e:
                    return f"解析失败: {e}"

            elif msg_type == "nav_status":
                try:
                    msg = GoalStatusArray.from_bytes(serialized_data)
                    if msg.status_list:
                        status_names = {0: "PENDING", 1: "ACTIVE", 3: "SUCCEEDED", 4: "ABORTED"}
                        status = msg.status_list[0]
                        status_name = status_names.get(status.status, f"UNKNOWN")
                        return f"{status_name}"
                    else:
                        return f"空状态"
                except Exception as e:
                    return f"解析失败: {e}"

            elif msg_type == "goal":
                try:
                    msg = PoseStamped.from_bytes(serialized_data)
                    return f"x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}"
                except Exception as e:
                    return f"解析失败: {e}"

            elif msg_type == "map":
                try:
                    msg = OccupancyGrid.from_bytes(serialized_data)
                    return f"{msg.info.width}x{msg.info.height}"
                except Exception as e:
                    return f"解析失败: {e}"

            else:
                return f"未知类型"

        except Exception as e:
            return f"解析消息失败: {e}"

    def send_goal_command(self):
        """发送导航目标"""
        try:
            x, y = 3.0, -1.5
            qz, qw = -0.7, 0.7
            goal_data = self.create_pose_stamped_data(x=x, y=y, qz=qz, qw=qw)

            if self.send_zmq_message(self.goal_sender, goal_data):
                print(f"发送目标: x={x:.1f}, y={y:.1f}")
            else:
                print("发送目标失败")
        except Exception as e:
            print(f"目标发送错误: {e}")

    def send_cmd_vel_command(self):
        """发送速度指令"""
        count = 0
        while self.running:
            try:
                count += 1
                linear_x = 0.3 + 0.1 * (count % 3)
                angular_z = 0.2 * (count % 2 - 0.5)
                cmd_vel_data = self.create_twist_data(linear_x=linear_x, angular_z=angular_z)

                if self.send_zmq_message(self.cmd_vel_sender, cmd_vel_data):
                    print(f"发送速度: v={linear_x:.2f}, w={angular_z:.2f}")
                else:
                    print("发送速度失败")
                time.sleep(2)
            except Exception as e:
                print(f"速度发送错误: {e}")
                time.sleep(1)

    def receive_robot_data(self):
        """接收机器人数据"""
        poller = zmq.Poller()
        poller.register(self.map_receiver, zmq.POLLIN)
        poller.register(self.amcl_receiver, zmq.POLLIN)
        poller.register(self.goal_receiver, zmq.POLLIN)
        poller.register(self.status_receiver, zmq.POLLIN)
        poller.register(self.result_receiver, zmq.POLLIN)

        while self.running:
            try:
                socks = dict(poller.poll(1000))

                if self.map_receiver in socks:
                    try:
                        message = self.map_receiver.recv_multipart()
                    except:
                        message = self.map_receiver.recv()
                    self.recv_counts['map'] += 1
                    if self.recv_counts['map'] % 5 == 0:
                        print(f"[地图 #{self.recv_counts['map']}]")

                if self.amcl_receiver in socks:
                    try:
                        message = self.amcl_receiver.recv_multipart()
                    except:
                        message = self.amcl_receiver.recv()
                    self.recv_counts['amcl_pose'] += 1
                    if self.recv_counts['amcl_pose'] % 10 == 0:
                        parsed = self.parse_zmq_message(message, msg_type="amcl_pose")
                        print(f"[位置 #{self.recv_counts['amcl_pose']}] {parsed}")

                if self.goal_receiver in socks:
                    try:
                        message = self.goal_receiver.recv_multipart()
                    except:
                        message = self.goal_receiver.recv()
                    self.recv_counts['current_goal'] += 1
                    parsed = self.parse_zmq_message(message, msg_type="goal")
                    print(f"[目标 #{self.recv_counts['current_goal']}] {parsed}")

                if self.status_receiver in socks:
                    try:
                        message = self.status_receiver.recv_multipart()
                    except:
                        message = self.status_receiver.recv()
                    self.recv_counts['nav_status'] += 1
                    if self.recv_counts['nav_status'] % 20 == 0:
                        parsed = self.parse_zmq_message(message, msg_type="nav_status")
                        print(f"[状态 #{self.recv_counts['nav_status']}] {parsed}")

                if self.result_receiver in socks:
                    try:
                        message = self.result_receiver.recv_multipart()
                    except:
                        message = self.result_receiver.recv()
                    self.recv_counts['nav_result'] += 1
                    print(f"[结果 #{self.recv_counts['nav_result']}]")

            except zmq.Again:
                pass
            except Exception as e:
                print(f"接收错误: {e}")

    def run(self):
        """启动模拟器"""
        print("启动模拟器...")

        recv_thread = threading.Thread(target=self.receive_robot_data)
        recv_thread.daemon = True
        recv_thread.start()

        time.sleep(3)
        self.send_goal_command()
        print("按Ctrl+C停止")

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n停止中...")
            self.running = False

    def cleanup(self):
        """清理资源"""
        self.goal_sender.close()
        self.cmd_vel_sender.close()
        self.cancel_sender.close()
        self.map_receiver.close()
        self.amcl_receiver.close()
        self.goal_receiver.close()
        self.status_receiver.close()
        self.result_receiver.close()
        self.context.term()

if __name__ == '__main__':
    simulator = DispatchCenterSimulator()
    try:
        simulator.run()
    except Exception as e:
        print(f"异常: {e}")
    finally:
        simulator.cleanup()
        print("模拟器已停止")

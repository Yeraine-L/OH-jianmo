#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能机械臂控制脚本
实现机械臂的自主导航、物体抓取和整理功能
"""

import os
import sys
import time
import random
import math
import numpy as np

# 添加项目根目录到Python路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

class SmartRobotArm:
    """智能机械臂控制类"""
    
    def __init__(self, scene_path):
        """初始化智能机械臂控制器"""
        self.scene_path = scene_path
        self.robot_arms = []
        self.objects = []
        self.environment = None
        self.task_queue = []
        
        # 区域定义
        self.areas = {
            "rest_area": {"center": [5, 4], "size": [2.5, 3], "priority": 2},
            "study_area": {"center": [8.5, 10], "size": [1.5, 2], "priority": 1},
            "storage_area": {"center": [2, 1.5], "size": [2, 1.5], "priority": 3},
            "play_area": {"center": [1.6, 8.75], "size": [1.6, 1.25], "priority": 4}
        }
        
        # 物体分类
        self.object_categories = {
            "toys": {"destination": "storage_area", "priority": 3},
            "books": {"destination": "study_area", "priority": 2},
            "stationery": {"destination": "study_area", "priority": 1},
            "clothes": {"destination": "storage_area", "priority": 4}
        }
        
        self.initialize()
    
    def initialize(self):
        """初始化环境和机械臂"""
        try:
            import mujoco
            import mujoco_viewer
            
            # 加载场景
            self.model = mujoco.MjModel.from_xml_path(self.scene_path)
            self.data = mujoco.MjData(self.model)
            
            # 创建查看器
            self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)
            
            # 初始化机械臂
            self.initialize_robot_arms()
            
            # 初始化物体
            self.initialize_objects()
            
            print("环境初始化成功！")
            print(f"检测到 {len(self.robot_arms)} 个机械臂")
            print(f"检测到 {len(self.objects)} 个可操作物体")
            
            self.simulation_mode = "real"
            
        except ImportError as e:
            print(f"警告：缺少必要的库: {e}")
            print("进入模拟模式，展示脚本逻辑流程")
            
            # 初始化机械臂
            self.initialize_robot_arms()
            
            # 初始化物体
            self.initialize_objects()
            
            print("模拟环境初始化成功！")
            print(f"检测到 {len(self.robot_arms)} 个机械臂")
            print(f"检测到 {len(self.objects)} 个可操作物体")
            
            self.simulation_mode = "simulated"
            self.model = None
            self.data = None
            self.viewer = None
            
        except Exception as e:
            print(f"环境初始化失败: {e}")
            print("进入模拟模式，展示脚本逻辑流程")
            
            # 初始化机械臂
            self.initialize_robot_arms()
            
            # 初始化物体
            self.initialize_objects()
            
            print("模拟环境初始化成功！")
            print(f"检测到 {len(self.robot_arms)} 个机械臂")
            print(f"检测到 {len(self.objects)} 个可操作物体")
            
            self.simulation_mode = "simulated"
            self.model = None
            self.data = None
            self.viewer = None
    
    def initialize_robot_arms(self):
        """初始化机械臂"""
        # 机械臂1: 位于 (5, 5, 3.0)
        arm1 = {
            "id": 1,
            "name": "robotic_arm_1",
            "base_pos": [5, 5, 3.0],
            "joints": [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint"
            ],
            "gripper_joints": ["left_finger_joint", "right_finger_joint"],
            "workspace": [
                [0, 10],  # x范围 - 覆盖整个房间
                [0, 12],  # y范围 - 覆盖整个房间
                [0, 2]   # z范围
            ],
            "current_task": None,
            "status": "idle"
        }
        
        # 机械臂2: 位于 (2, 9, 3.0)
        arm2 = {
            "id": 2,
            "name": "robotic_arm_2",
            "base_pos": [2, 9, 3.0],
            "joints": [
                "shoulder_pan_joint_2",
                "shoulder_lift_joint_2",
                "elbow_joint_2",
                "wrist_1_joint_2",
                "wrist_2_joint_2",
                "wrist_3_joint_2"
            ],
            "gripper_joints": ["left_finger_joint_2", "right_finger_joint_2"],
            "workspace": [
                [0, 10],  # x范围 - 覆盖整个房间
                [0, 12],  # y范围 - 覆盖整个房间
                [0, 2]   # z范围
            ],
            "current_task": None,
            "status": "idle"
        }
        
        self.robot_arms = [arm1, arm2]
    
    def initialize_objects(self):
        """初始化物体列表"""
        # 从场景中检测物体
        # 这里我们根据场景文件中的物体位置和类型来初始化
        
        # 游戏区玩具
        play_toys = [
            {"name": "ball_01", "type": "toys", "pos": [0.5, 8.5, 0.1], "priority": 3},
            {"name": "ball_02", "type": "toys", "pos": [1, 8.5, 0.08], "priority": 3},
            {"name": "ball_03", "type": "toys", "pos": [1.5, 8.5, 0.12], "priority": 3},
            {"name": "block_01", "type": "toys", "pos": [0.5, 9, 0.08], "priority": 3},
            {"name": "block_02", "type": "toys", "pos": [0.7, 9, 0.08], "priority": 3},
            {"name": "block_03", "type": "toys", "pos": [0.9, 9, 0.08], "priority": 3},
            {"name": "plush_01", "type": "toys", "pos": [0.75, 9.25, 0.15], "priority": 3},
            {"name": "car_01", "type": "toys", "pos": [0.5, 9.5, 0.05], "priority": 3}
        ]
        
        # 存储区物体
        storage_toys = [
            {"name": "st_toy_01", "type": "toys", "pos": [0.3, 0.3, 0.08], "priority": 3},
            {"name": "st_toy_02", "type": "toys", "pos": [1.0, 0.3, 0.08], "priority": 3},
            {"name": "st_toy_03", "type": "toys", "pos": [1.7, 0.3, 0.08], "priority": 3}
        ]
        
        # 学习区物体
        study_objects = [
            {"name": "monitor", "type": "stationery", "pos": [8.5, 10, 0.47], "priority": 1},
            {"name": "keyboard", "type": "stationery", "pos": [8.5, 10, 0.38], "priority": 1},
            {"name": "mouse", "type": "stationery", "pos": [8.5, 10, 0.38], "priority": 1}
        ]
        
        # 合并所有物体
        self.objects = play_toys + storage_toys + study_objects
        
        # 生成任务队列
        self.generate_task_queue()
    
    def generate_task_queue(self):
        """生成任务队列"""
        for obj in self.objects:
            # 确定物体的目标位置
            category_info = self.object_categories.get(obj["type"], {"destination": "storage_area", "priority": 4})
            destination = category_info["destination"]
            
            # 确定任务优先级
            task_priority = category_info["priority"]
            
            # 创建任务
            task = {
                "id": f"task_{len(self.task_queue) + 1}",
                "object": obj,
                "source": self.get_object_area(obj["pos"]),
                "destination": destination,
                "priority": task_priority,
                "status": "pending",
                "assigned_arm": None
            }
            
            self.task_queue.append(task)
        
        # 按优先级排序任务队列
        self.task_queue.sort(key=lambda x: x["priority"])
        
        print(f"生成了 {len(self.task_queue)} 个任务")
    
    def get_object_area(self, pos):
        """确定物体所在区域"""
        for area_name, area_info in self.areas.items():
            center = area_info["center"]
            size = area_info["size"]
            
            if (center[0] - size[0]/2 <= pos[0] <= center[0] + size[0]/2 and
                center[1] - size[1]/2 <= pos[1] <= center[1] + size[1]/2):
                return area_name
        
        return "unknown"
    
    def assign_task(self, arm):
        """为机械臂分配任务"""
        # 找到适合该机械臂的任务
        best_task = None
        best_score = float('inf')
        
        for task in self.task_queue:
            if task["status"] == "pending":
                # 检查任务是否在机械臂的工作范围内
                if self.is_task_in_arm_workspace(arm, task):
                    # 计算任务评分（优先级 + 距离）
                    obj_pos = task["object"]["pos"]
                    arm_pos = arm["base_pos"]
                    distance = math.sqrt(
                        (obj_pos[0] - arm_pos[0])**2 +
                        (obj_pos[1] - arm_pos[1])**2 +
                        (obj_pos[2] - arm_pos[2])**2
                    )
                    
                    # 计算任务评分：优先级（越小越好） + 距离（越小越好）
                    score = task["priority"] + distance * 0.1
                    
                    if score < best_score:
                        best_score = score
                        best_task = task
        
        if best_task:
            best_task["status"] = "assigned"
            best_task["assigned_arm"] = arm["id"]
            arm["current_task"] = best_task
            arm["status"] = "working"
            print(f"为机械臂 {arm['id']} 分配任务: {best_task['id']} (评分: {best_score:.2f})")
            return True
        
        return False
    
    def is_task_in_arm_workspace(self, arm, task):
        """检查任务是否在机械臂的工作范围内"""
        obj_pos = task["object"]["pos"]
        workspace = arm["workspace"]
        
        return (workspace[0][0] <= obj_pos[0] <= workspace[0][1] and
                workspace[1][0] <= obj_pos[1] <= workspace[1][1] and
                workspace[2][0] <= obj_pos[2] <= workspace[2][1])
    
    def move_arm_to_position(self, arm_id, target_pos, gripper_open=True):
        """移动机械臂到指定位置"""
        arm = next((a for a in self.robot_arms if a["id"] == arm_id), None)
        if not arm:
            return False
        
        try:
            # 路径规划
            path = self.plan_path(arm_id, self.get_current_arm_position(arm_id), target_pos)
            if not path:
                print(f"无法规划路径到目标位置: {target_pos}")
                return False
            
            # 打开或关闭夹爪
            if gripper_open:
                self.open_gripper(arm_id)
            else:
                self.close_gripper(arm_id)
            
            # 沿规划路径移动
            for waypoint in path:
                # 检查碰撞
                if self.check_collision(arm_id, waypoint):
                    print(f"路径点 {waypoint} 存在碰撞风险，重新规划路径")
                    # 重新规划路径
                    new_path = self.plan_path(arm_id, self.get_current_arm_position(arm_id), target_pos)
                    if not new_path:
                        return False
                    path = new_path
                    break
                
                # 移动到路径点
                self.move_to_waypoint(arm_id, waypoint, gripper_open)
            
            return True
            
        except Exception as e:
            print(f"移动机械臂失败: {e}")
            return False
    
    def get_current_arm_position(self, arm_id):
        """获取机械臂末端当前位置"""
        arm = next((a for a in self.robot_arms if a["id"] == arm_id), None)
        if not arm:
            return [0, 0, 0]
        
        # 简化实现：返回机械臂基座位置
        return arm["base_pos"]
    
    def plan_path(self, arm_id, start_pos, target_pos):
        """规划机械臂路径"""
        # 改进的路径规划：考虑碰撞避免
        path = []
        
        # 计算路径点
        num_waypoints = 10
        for i in range(num_waypoints + 1):
            t = i / num_waypoints
            waypoint = [
                start_pos[0] + t * (target_pos[0] - start_pos[0]),
                start_pos[1] + t * (target_pos[1] - start_pos[1]),
                start_pos[2] + t * (target_pos[2] - start_pos[2])
            ]
            
            # 检查碰撞
            if self.check_collision(arm_id, waypoint):
                # 碰撞避免：抬升机械臂
                waypoint[2] = max(waypoint[2], 1.0)  # 抬升到至少 1 米高度
                print(f"[路径规划] 检测到碰撞风险，抬升机械臂到高度: {waypoint[2]}")
            
            path.append(waypoint)
        
        return path
    
    def check_collision(self, arm_id, target_pos):
        """检查碰撞"""
        # 简化的碰撞检查：检查是否与墙壁或其他物体碰撞
        
        # 检查墙壁碰撞
        if target_pos[0] <= 0.1 or target_pos[0] >= 9.9:
            return True
        if target_pos[1] <= 0.1 or target_pos[1] >= 11.9:
            return True
        if target_pos[2] <= 0:
            return True
        
        # 检查与其他物体的碰撞
        for obj in self.objects:
            obj_pos = obj["pos"]
            distance = math.sqrt(
                (target_pos[0] - obj_pos[0])**2 +
                (target_pos[1] - obj_pos[1])**2 +
                (target_pos[2] - obj_pos[2])**2
            )
            if distance < 0.2:  # 碰撞阈值
                return True
        
        return False
    
    def move_to_waypoint(self, arm_id, waypoint, gripper_open=True):
        """移动到路径点"""
        # 简化实现：直接移动到目标点
        # 实际应用中需要实现逆向运动学
        
        if self.simulation_mode == "real" and self.model and self.data:
            # 模拟移动过程
            try:
                import mujoco
                for i in range(20):
                    # 更新数据
                    mujoco.mj_step(self.model, self.data)
                    
                    # 渲染
                    if self.viewer:
                        self.viewer.render()
                    
                    time.sleep(0.005)
            except Exception as e:
                print(f"移动到路径点失败: {e}")
        else:
            # 模拟模式
            print(f"[模拟] 机械臂 {arm_id} 移动到路径点: {waypoint}")
            time.sleep(0.05)
    
    def open_gripper(self, arm_id):
        """打开夹爪"""
        arm = next((a for a in self.robot_arms if a["id"] == arm_id), None)
        if not arm:
            return
        
        if self.simulation_mode == "real" and self.model and self.data:
            # 找到夹爪关节并设置为打开状态
            try:
                import mujoco
                for joint_name in arm["gripper_joints"]:
                    joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                    if joint_id != -1:
                        self.data.ctrl[joint_id] = 0.08  # 打开夹爪
            except Exception as e:
                print(f"打开夹爪失败: {e}")
        else:
            # 模拟模式
            print(f"[模拟] 机械臂 {arm_id} 打开夹爪")
    
    def close_gripper(self, arm_id):
        """关闭夹爪"""
        arm = next((a for a in self.robot_arms if a["id"] == arm_id), None)
        if not arm:
            return
        
        if self.simulation_mode == "real" and self.model and self.data:
            # 找到夹爪关节并设置为关闭状态
            try:
                import mujoco
                for joint_name in arm["gripper_joints"]:
                    joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                    if joint_id != -1:
                        self.data.ctrl[joint_id] = 0.0  # 关闭夹爪
            except Exception as e:
                print(f"关闭夹爪失败: {e}")
        else:
            # 模拟模式
            print(f"[模拟] 机械臂 {arm_id} 关闭夹爪")
    
    def grasp_object(self, arm_id, obj):
        """抓取物体"""
        print(f"机械臂 {arm_id} 尝试抓取物体: {obj['name']}")
        
        # 移动到物体上方
        target_pos = obj["pos"]
        target_pos[2] += 0.1  # 高于物体
        
        # 移动到物体上方
        if not self.move_arm_to_position(arm_id, target_pos, gripper_open=True):
            return False
        
        # 移动到物体位置
        target_pos[2] = obj["pos"][2] + 0.05  # 接近物体
        if not self.move_arm_to_position(arm_id, target_pos, gripper_open=True):
            return False
        
        # 关闭夹爪
        self.close_gripper(arm_id)
        
        # 提升物体
        target_pos[2] += 0.2  # 提升物体
        if not self.move_arm_to_position(arm_id, target_pos, gripper_open=False):
            return False
        
        print(f"机械臂 {arm_id} 成功抓取物体: {obj['name']}")
        return True
    
    def place_object(self, arm_id, destination):
        """放置物体"""
        arm = next((a for a in self.robot_arms if a["id"] == arm_id), None)
        if not arm or not arm["current_task"]:
            return False
        
        print(f"机械臂 {arm_id} 尝试放置物体到: {destination}")
        
        # 确定目标位置
        area_info = self.areas.get(destination, {"center": [5, 5], "size": [1, 1]})
        target_pos = [
            area_info["center"][0] + random.uniform(-area_info["size"][0]/4, area_info["size"][0]/4),
            area_info["center"][1] + random.uniform(-area_info["size"][1]/4, area_info["size"][1]/4),
            0.1  # 放置高度
        ]
        
        # 移动到目标位置上方
        target_pos[2] += 0.2  # 高于目标位置
        if not self.move_arm_to_position(arm_id, target_pos, gripper_open=False):
            return False
        
        # 移动到目标位置
        target_pos[2] = 0.1  # 放置高度
        if not self.move_arm_to_position(arm_id, target_pos, gripper_open=False):
            return False
        
        # 打开夹爪
        self.open_gripper(arm_id)
        
        # 后退
        target_pos[2] += 0.2  # 提升
        if not self.move_arm_to_position(arm_id, target_pos, gripper_open=True):
            return False
        
        print(f"机械臂 {arm_id} 成功放置物体到: {destination}")
        return True
    
    def execute_task(self, arm):
        """执行任务"""
        if not arm["current_task"]:
            return False
        
        task = arm["current_task"]
        obj = task["object"]
        
        print(f"执行任务: {task['id']} - 移动 {obj['name']} 从 {task['source']} 到 {task['destination']}")
        
        # 抓取物体
        if not self.grasp_object(arm["id"], obj):
            task["status"] = "failed"
            arm["status"] = "idle"
            arm["current_task"] = None
            return False
        
        # 放置物体
        if not self.place_object(arm["id"], task["destination"]):
            task["status"] = "failed"
            arm["status"] = "idle"
            arm["current_task"] = None
            return False
        
        # 任务完成
        task["status"] = "completed"
        arm["status"] = "idle"
        arm["current_task"] = None
        
        print(f"任务完成: {task['id']}")
        return True
    
    def run(self):
        """运行主循环"""
        try:
            while True:
                # 分配任务
                for arm in self.robot_arms:
                    if arm["status"] == "idle":
                        self.assign_task(arm)
                
                # 执行任务
                for arm in self.robot_arms:
                    if arm["status"] == "working":
                        self.execute_task(arm)
                
                # 检查是否所有任务都已完成
                if all(task["status"] in ["completed", "failed"] for task in self.task_queue):
                    print("所有任务已完成！")
                    break
                
                # 模拟环境
                if self.simulation_mode == "real" and self.model and self.data:
                    try:
                        import mujoco
                        mujoco.mj_step(self.model, self.data)
                        
                        # 渲染
                        if self.viewer:
                            self.viewer.render()
                    except Exception as e:
                        print(f"模拟环境失败: {e}")
                else:
                    # 模拟模式
                    print("[模拟] 运行主循环")
                    # 模拟时间步
                    time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("程序被用户中断")
        finally:
            if self.simulation_mode == "real" and hasattr(self, "viewer") and self.viewer:
                try:
                    self.viewer.close()
                except Exception as e:
                    print(f"关闭查看器失败: {e}")
    
    def get_statistics(self):
        """获取统计信息"""
        completed_tasks = [t for t in self.task_queue if t["status"] == "completed"]
        failed_tasks = [t for t in self.task_queue if t["status"] == "failed"]
        pending_tasks = [t for t in self.task_queue if t["status"] == "pending"]
        working_tasks = [t for t in self.task_queue if t["status"] == "assigned"]
        
        stats = {
            "total_tasks": len(self.task_queue),
            "completed_tasks": len(completed_tasks),
            "failed_tasks": len(failed_tasks),
            "pending_tasks": len(pending_tasks),
            "working_tasks": len(working_tasks),
            "completion_rate": len(completed_tasks) / len(self.task_queue) * 100 if self.task_queue else 0
        }
        
        return stats

if __name__ == "__main__":
    # 场景文件路径
    scene_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scenes", "kids_room_mujoco.1.26.xml")
    
    # 创建智能机械臂控制器
    robot_controller = SmartRobotArm(scene_path)
    
    # 运行
    robot_controller.run()
    
    # 打印统计信息
    stats = robot_controller.get_statistics()
    print("\n任务完成统计:")
    for key, value in stats.items():
        print(f"{key}: {value}")

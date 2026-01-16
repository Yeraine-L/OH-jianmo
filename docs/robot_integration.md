# 机器人集成指南

本指南将介绍如何将机器人模型集成到 MuJoCo 仿真场景中。

---

## 概述

项目支持三种主要的机器人类型集成：
1. **机械臂** (Robotic Arms)
2. **人形机器人** (Humanoid Robots)
3. **移动机器人** (Mobile Robots)

每种类型的集成方式略有不同，但核心思路一致：使用 MuJoCo 的 `<include>` 标签将机器人模型引入场景。

---

## 集成方法

### 方法 1：使用 `<include>` 标签（推荐）

这是最灵活、可维护性最高的方法。

**步骤**：

1. 将机器人模型文件放置在 `models/robots/` 目录下
2. 创建新的场景 XML 文件，引用基础场景和机器人模型
3. 在 `<worldbody>` 中定位机器人位置

**示例**：将 UR5 机械臂放置在客厅场景

```xml
<mujoco model="living_room_with_ur5">
  <compiler angle="degree" coordinate="local"/>

  <!-- 引入机器人模型 -->
  <include file="../models/robots/ur5/ur5_simplified.xml"/>

  <!-- 场景主体 -->
  <worldbody>
    <!-- 引入客厅基础场景 -->
    <include file="living_room.xml"/>

    <!-- 放置机器人 -->
    <body name="ur5_base" pos="-1.5 -1.5 0.8" euler="0 0 0">
      <!-- 机器人模型会自动插入到这里 -->
    </body>
  </worldbody>
</mujoco>
```

### 方法 2：直接嵌入模型

适用于简单场景或需要高度定制的情况。

直接在场景文件的 `<worldbody>` 中定义机器人结构。

---

## 机械臂集成

### UR5 机械臂示例

**模型特点**：
- 6 自由度
- 工作空间半径：约 850mm
- 有效载荷：5kg

**集成步骤**：

1. **准备模型文件**

   将 UR5 模型放置在：
   ```
   models/robots/ur5/
   ├── ur5_simplified.xml      # 简化版模型
   ├── ur5_full.xml            # 完整版模型（含详细网格）
   └── meshes/                 # 视觉网格文件
   ```

2. **创建集成场景**

   ```xml
   <mujoco model="living_room_with_ur5">
     <include file="../models/robots/ur5/ur5_simplified.xml"/>

     <worldbody>
       <!-- 基础场景 -->
       <geom name="floor" type="plane" size="3 3 0.1"/>

       <!-- UR5 安装在桌面上 -->
       <body name="ur5_mount" pos="0 0 0.4">
         <geom name="mount_plate" type="cylinder"
               size="0.15 0.02" rgba="0.3 0.3 0.3 1"/>

         <!-- 机械臂基座 -->
         <body name="ur5_base" pos="0 0 0.02">
           <!-- UR5 关节链 -->
           <joint name="shoulder_pan" type="hinge" axis="0 0 1"
                  range="-180 180" damping="0.5"/>
           <!-- 更多关节定义... -->
         </body>
       </body>
     </worldbody>

     <!-- 执行器配置 -->
     <actuator>
       <motor name="shoulder_pan_motor" joint="shoulder_pan"
              gear="100" ctrllimited="true" ctrlrange="-2 2"/>
       <!-- 更多执行器... -->
     </actuator>
   </mujoco>
   ```

3. **测试集成**

   ```python
   import mujoco
   import mujoco.viewer

   model = mujoco.MjModel.from_xml_path("scenes/living_room_with_ur5.xml")
   data = mujoco.MjData(model)

   with mujoco.viewer.launch_passive(model, data) as viewer:
       while viewer.is_running():
           # 设置关节目标位置（示例）
           data.ctrl[0] = 0.5  # shoulder_pan

           mujoco.mj_step(model, data)
           viewer.sync()
   ```

### Franka Panda 机械臂

**模型特点**：
- 7 自由度（冗余臂）
- 工作空间半径：855mm
- 有效载荷：3kg

集成方式类似，但需要注意 7 自由度带来的更复杂的运动学。

---

## 人形机器人集成

### 基本结构

人形机器人通常包含：
- 躯干（torso）
- 双腿（left_leg, right_leg）
- 双臂（left_arm, right_arm）
- 头部（head）

**集成示例**：

```xml
<mujoco model="living_room_with_humanoid">
  <worldbody>
    <!-- 场景 -->
    <include file="living_room.xml"/>

    <!-- 人形机器人 -->
    <body name="humanoid_pelvis" pos="-1.5 -1.5 1.0">
      <freejoint name="root"/>  <!-- 自由浮动基座 -->

      <geom name="torso" type="capsule" size="0.1 0.3"
            fromto="0 0 0 0 0 0.6" rgba="0.7 0.7 0.7 1"/>

      <!-- 左腿 -->
      <body name="left_thigh" pos="-0.1 0 -0.1">
        <joint name="left_hip" type="hinge" axis="1 0 0" range="-90 90"/>
        <geom name="left_thigh_geom" type="capsule" size="0.05 0.2"
              fromto="0 0 0 0 0 -0.4"/>
        <!-- 小腿、脚... -->
      </body>

      <!-- 右腿（镜像） -->
      <!-- ... -->
    </body>
  </worldbody>

  <!-- 执行器 -->
  <actuator>
    <motor name="left_hip_motor" joint="left_hip" gear="150"/>
    <!-- 更多执行器... -->
  </actuator>
</mujoco>
```

### 稳定性建议

人形机器人在仿真中容易失稳，建议：

1. **调整时间步长**：使用更小的 timestep（如 0.001）
2. **增加阻尼**：在关节上设置合理的 `damping` 参数
3. **使用 PD 控制器**：
   ```xml
   <actuator>
     <position name="left_hip_pos" joint="left_hip"
               kp="100" kv="10" ctrlrange="-1.57 1.57"/>
   </actuator>
   ```
4. **初始姿态**：确保初始位置稳定（双脚着地）

---

## 移动机器人集成

### 轮式小车

**示例：差分驱动小车**

```xml
<body name="robot_base" pos="0 0 0.1">
  <freejoint name="base_joint"/>

  <!-- 车身 -->
  <geom name="chassis" type="box" size="0.2 0.15 0.05"
        rgba="0.2 0.2 0.8 1" mass="2"/>

  <!-- 左轮 -->
  <body name="left_wheel" pos="-0.15 0.2 0">
    <joint name="left_wheel_joint" type="hinge" axis="0 1 0"/>
    <geom name="left_wheel_geom" type="cylinder"
          size="0.05 0.02" rgba="0.1 0.1 0.1 1" mass="0.2"/>
  </body>

  <!-- 右轮 -->
  <body name="right_wheel" pos="-0.15 -0.2 0">
    <joint name="right_wheel_joint" type="hinge" axis="0 1 0"/>
    <geom name="right_wheel_geom" type="cylinder"
          size="0.05 0.02" rgba="0.1 0.1 0.1 1" mass="0.2"/>
  </body>

  <!-- 万向轮 -->
  <body name="caster" pos="0.15 0 -0.03">
    <geom name="caster_geom" type="sphere" size="0.02"
          rgba="0.3 0.3 0.3 1" mass="0.05"/>
  </body>
</body>

<!-- 电机执行器 -->
<actuator>
  <motor name="left_motor" joint="left_wheel_joint" gear="10"/>
  <motor name="right_motor" joint="right_wheel_joint" gear="10"/>
</actuator>
```

---

## 高级技巧

### 1. 传感器配置

为机器人添加传感器（IMU、关节编码器等）：

```xml
<sensor>
  <!-- IMU -->
  <accelerometer name="imu_accel" site="imu_site"/>
  <gyro name="imu_gyro" site="imu_site"/>

  <!-- 关节位置 -->
  <jointpos name="shoulder_pos" joint="shoulder_pan"/>

  <!-- 触觉传感器 -->
  <touch name="gripper_touch" site="gripper_tip"/>
</sensor>
```

### 2. 末端执行器

为机械臂添加夹爪：

```xml
<body name="gripper" pos="0 0 0.1">
  <!-- 左指 -->
  <body name="left_finger" pos="0 0.02 0">
    <joint name="left_finger_joint" type="slide" axis="0 1 0"
           range="0 0.04"/>
    <geom name="left_finger_geom" type="box" size="0.01 0.005 0.03"/>
  </body>

  <!-- 右指（镜像） -->
  <body name="right_finger" pos="0 -0.02 0">
    <joint name="right_finger_joint" type="slide" axis="0 -1 0"
           range="0 0.04"/>
    <geom name="right_finger_geom" type="box" size="0.01 0.005 0.03"/>
  </body>
</body>

<!-- 夹爪执行器 -->
<actuator>
  <motor name="gripper_motor" joint="left_finger_joint" gear="20"/>
  <motor name="gripper_motor_mirror" joint="right_finger_joint" gear="20"/>
</actuator>
```

### 3. 碰撞过滤

避免机器人自身部件碰撞：

```xml
<geom name="link1" contype="1" conaffinity="2"/>
<geom name="link2" contype="2" conaffinity="1"/>
```

---

## 常见问题

### Q1：机器人模型加载失败

**检查清单**：
- 文件路径是否正确（相对路径 vs 绝对路径）
- XML 语法是否有误
- 是否缺少必要的 `<compiler>` 设置

### Q2：机器人位置不正确

- 检查 `pos` 和 `euler`/`quat` 参数
- 确保坐标系一致（`coordinate="local"` vs `"global"`）
- 使用可视化工具查看坐标轴方向

### Q3：关节运动异常

- 检查 `joint` 的 `range` 参数
- 调整 `damping` 和 `armature` 减少震荡
- 确保执行器 `gear` 比例合理

### Q4：仿真不稳定

- 减小 `timestep`（如从 0.002 改为 0.001）
- 增加 `solver` 迭代次数：
  ```xml
  <option iterations="50" tolerance="1e-10"/>
  ```
- 检查质量分布是否合理

---

## 下一步

- 查看 `models/robots/` 目录下的示例模型
- 运行 `scripts/test_robot_integration.py` 测试集成
- 参考 [API 文档](api_reference.md) 了解编程接口

---

**更新日期**: 2026-01-15

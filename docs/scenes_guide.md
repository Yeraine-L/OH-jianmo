# 场景使用指南

本文档提供 MuJoCo 仿真场景的详细使用指南。

---

## 快速开始

### 1. 环境设置

确保已安装所有依赖：

```bash
pip install -r requirements.txt
```

### 2. 运行第一个场景

使用提供的测试脚本：

```bash
python scripts/test_scene.py
```

这将加载并可视化 `living_room.xml` 场景。

### 3. 可视化操作

**鼠标控制**：
- 左键拖动：旋转视角
- 右键拖动：平移视角
- 滚轮：缩放
- 双击：聚焦到点击的对象

**键盘快捷键**（在查看器中）：
- `空格`：暂停/恢复仿真
- `Ctrl+P`：暂停仿真
- `Ctrl+R`：重置仿真
- `F5`：重新加载模型
- `ESC`：退出

---

## 场景列表

### 家居场景

#### living_room.xml

**描述**：标准客厅环境

**尺寸**：6m × 6m × 2.5m

**包含元素**：
- 地板（棋盘格纹理）
- 四面墙壁
- 中央桌子（1.2m × 0.8m）
- 沙发
- 地毯
- 机器人生成点标记

**适用于**：
- 移动机器人导航
- 物体抓取任务
- 服务机器人测试

**使用示例**：
```python
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("scenes/living_room.xml")
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
```

#### living_room_with_ur5.xml

**描述**：客厅场景 + UR5 机械臂工作站

**新增元素**：
- 工作台（0.8m × 0.8m）
- UR5 机械臂（6自由度）
- 简单夹爪
- 可操作立方体

**控制接口**：
```python
# 设置关节目标位置（弧度）
data.ctrl[0] = 0.5    # shoulder_pan
data.ctrl[1] = -0.3   # shoulder_lift
data.ctrl[2] = 0.8    # elbow
data.ctrl[3] = -0.5   # wrist_1
data.ctrl[4] = 0.2    # wrist_2
data.ctrl[5] = 0.0    # wrist_3

# 控制夹爪（0=关闭, 0.04=完全打开）
data.ctrl[6] = 0.02   # gripper
data.ctrl[7] = 0.02   # gripper_mirror
```

**适用于**：
- 机械臂抓取算法
- 逆运动学测试
- 轨迹规划

#### living_room_with_humanoid.xml

**描述**：客厅场景 + 简化人形机器人

**机器人规格**：
- 身高：约 1.7m
- 自由度：14+ DOF
- 质量：约 35kg

**控制提示**：
```python
# 简单的站立平衡控制
# 需要实现 PD 控制器或强化学习策略

# 读取传感器数据
accel = data.sensordata[0:3]    # 加速度计
gyro = data.sensordata[3:6]     # 陀螺仪
foot_force_r = data.sensordata[6:9]  # 右脚力传感器
foot_force_l = data.sensordata[9:12] # 左脚力传感器
```

**适用于**：
- 双足步态研究
- 平衡控制算法
- 全身运动规划

---

## 自定义场景

### 修改现有场景

1. **调整房间尺寸**

   编辑地板和墙壁的 `size` 参数：
   ```xml
   <!-- 将房间从 6m×6m 改为 8m×8m -->
   <geom name="floor" type="plane" size="4 4 0.1"/>
   ```

2. **添加新家具**

   在 `<worldbody>` 中插入新的 `<body>` 元素：
   ```xml
   <body name="chair" pos="1 1 0.25">
     <geom name="seat" type="box" size="0.2 0.2 0.05"
           rgba="0.5 0.3 0.2 1" mass="5"/>
     <geom name="backrest" type="box" pos="0 0.15 0.3"
           size="0.2 0.05 0.3" rgba="0.5 0.3 0.2 1" mass="2"/>
   </body>
   ```

3. **更改材质和纹理**

   修改 `<asset>` 部分：
   ```xml
   <texture name="custom_floor" type="2d" builtin="gradient"
            rgb1="0.2 0.3 0.5" rgb2="0.1 0.2 0.3"/>
   <material name="custom_mat" texture="custom_floor"
             specular="0.5" shininess="0.2"/>
   ```

### 创建全新场景

1. **从模板开始**

   复制 `living_room.xml` 并重命名
   ```bash
   cp scenes/living_room.xml scenes/my_scene.xml
   ```

2. **定义场景结构**

   最小化场景模板：
   ```xml
   <mujoco model="my_scene">
     <compiler angle="degree"/>
     <option timestep="0.002" gravity="0 0 -9.81"/>

     <asset>
       <!-- 材质和纹理定义 -->
     </asset>

     <worldbody>
       <!-- 地板 -->
       <geom name="floor" type="plane" size="10 10 0.1"/>

       <!-- 其他对象 -->
     </worldbody>
   </mujoco>
   ```

3. **验证场景**

   使用测试脚本检查：
   ```bash
   python scripts/test_scene.py
   ```

---

## 物理参数调整

### 摩擦系数

控制物体之间的摩擦：

```xml
<!-- friction="滑动 滚动 扭转" -->
<geom name="floor" friction="1.0 0.5 0.01"/>

<!-- 高摩擦（如橡胶） -->
<geom name="rubber_mat" friction="1.5 0.8 0.02"/>

<!-- 低摩擦（如冰面） -->
<geom name="ice" friction="0.1 0.05 0.001"/>
```

### 质量和惯性

```xml
<!-- 显式指定质量 -->
<geom name="object" mass="2.5"/>

<!-- 自动从几何计算（需要设置密度） -->
<geom name="object" density="1000"/>
```

### 碰撞检测

```xml
<!-- 禁用碰撞（如装饰物） -->
<geom name="decoration" contype="0" conaffinity="0"/>

<!-- 分组碰撞（避免自碰撞） -->
<geom name="robot_link1" contype="1" conaffinity="2"/>
<geom name="robot_link2" contype="2" conaffinity="1"/>
```

---

## 光照和视觉效果

### 添加光源

```xml
<worldbody>
  <!-- 点光源 -->
  <light name="spotlight" pos="2 2 3" dir="-1 -1 -1"
         directional="false" diffuse="1 1 1" specular="0.5 0.5 0.5"/>

  <!-- 平行光（模拟太阳） -->
  <light name="sun" pos="0 0 10" dir="0 0 -1"
         directional="true" diffuse="0.8 0.8 0.8"/>
</worldbody>
```

### 调整全局视觉效果

```xml
<visual>
  <!-- 渲染分辨率 -->
  <global offwidth="1920" offheight="1080"/>

  <!-- 阴影质量 -->
  <quality shadowsize="4096"/>

  <!-- 雾效和视距 -->
  <map fog="0.3" znear="0.01" zfar="50"/>

  <!-- 头灯亮度 -->
  <headlight ambient="0.3 0.3 0.3" diffuse="0.7 0.7 0.7"/>
</visual>
```

---

## 传感器配置

### 常用传感器

```xml
<sensor>
  <!-- 关节位置 -->
  <jointpos name="joint_pos" joint="my_joint"/>

  <!-- 关节速度 -->
  <jointvel name="joint_vel" joint="my_joint"/>

  <!-- 关节力矩 -->
  <torque name="joint_torque" joint="my_joint"/>

  <!-- 接触力 -->
  <touch name="contact" site="my_site"/>

  <!-- IMU -->
  <accelerometer name="accel" site="imu_site"/>
  <gyro name="gyro" site="imu_site"/>

  <!-- 位置和姿态 -->
  <framepos name="ee_pos" objtype="site" objname="ee_site"/>
  <framequat name="ee_quat" objtype="site" objname="ee_site"/>
</sensor>
```

### 读取传感器数据

```python
# 获取传感器数据
sensor_data = data.sensordata

# 按名称访问
sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "joint_pos")
value = data.sensordata[model.sensor_adr[sensor_id]]
```

---

## 性能优化

### 提高仿真速度

1. **减少碰撞对**
   ```xml
   <geom contype="0" conaffinity="0"/>  <!-- 禁用不必要的碰撞 -->
   ```

2. **增大时间步长**（可能影响稳定性）
   ```xml
   <option timestep="0.005"/>  <!-- 从 0.002 增加到 0.005 -->
   ```

3. **减少求解器迭代**
   ```xml
   <option iterations="30"/>  <!-- 默认 50 -->
   ```

### 提高渲染性能

1. **降低阴影质量**
   ```xml
   <quality shadowsize="2048"/>  <!-- 从 4096 降低 -->
   ```

2. **使用无头模式**（批量仿真）
   ```python
   import mujoco

   model = mujoco.MjModel.from_xml_path("scene.xml")
   data = mujoco.MjData(model)

   # 无渲染循环
   for i in range(10000):
       mujoco.mj_step(model, data)
   ```

---

## 故障排除

### 常见问题

**Q: 模型加载失败**
```
Error: Failed to load XML
```
**解决**：
- 检查 XML 语法（使用 XML 验证工具）
- 确保所有引用的文件路径正确
- 查看 MuJoCo 错误日志获取详细信息

**Q: 仿真不稳定（抖动、爆炸）**

**解决**：
- 减小时间步长：`<option timestep="0.001"/>`
- 增加求解器迭代：`<option iterations="100"/>`
- 检查质量分布是否合理
- 增加关节阻尼：`damping="5"`

**Q: 机器人倒下**

**解决**：
- 检查初始位置是否稳定
- 确保脚底有足够的摩擦力
- 实现平衡控制器
- 使用 `freejoint` 允许基座移动

---

## 导出与分享

### 录制视频

```python
import mujoco
import imageio

model = mujoco.MjModel.from_xml_path("scenes/living_room.xml")
data = mujoco.MjData(model)

# 离屏渲染
renderer = mujoco.Renderer(model, height=720, width=1280)

frames = []
for i in range(500):
    mujoco.mj_step(model, data)
    renderer.update_scene(data)
    frame = renderer.render()
    frames.append(frame)

# 保存为视频
imageio.mimsave("output.mp4", frames, fps=30)
```

### 导出模型信息

```python
# 打印模型统计
print(f"Bodies: {model.nbody}")
print(f"Joints: {model.njnt}")
print(f"Geoms: {model.ngeom}")
print(f"Actuators: {model.nu}")
```

---

## 下一步

- 查看 [机器人集成指南](robot_integration.md)
- 探索 [API 参考文档](api_reference.md)
- 尝试创建自定义任务

---

**最后更新**: 2026-01-15

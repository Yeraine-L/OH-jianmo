# MuJoCo Menagerie 模型集成指南

本指南介绍如何将 MuJoCo Menagerie 中的高质量机器人模型集成到您的自定义场景中。

---

## 什么是 MuJoCo Menagerie？

**MuJoCo Menagerie** 是 Google DeepMind 官方维护的高质量机器人模型库，包含：

- ✅ 工业机械臂（UR5e, UR10e, Franka Panda 等）
- ✅ 移动机器人（ANYmal, Unitree Go1, Spot 等）
- ✅ 人形机器人（Unitree H1, Shadow Hand 等）
- ✅ 完整的物理参数和视觉模型
- ✅ 经过验证的准确性

**GitHub**: https://github.com/google-deepmind/mujoco_menagerie

---

## 安装与设置

### 步骤 1：克隆 Menagerie 仓库

**推荐方式 - 项目本地安装**：

```bash
cd C:\Users\夜澜衣\Desktop\01
git clone https://github.com/google-deepmind/mujoco_menagerie.git menagerie
```

安装后的目录结构：
```
01/
├── menagerie/                  # MuJoCo Menagerie 仓库
│   ├── universal_robots_ur5e/
│   │   ├── ur5e.xml
│   │   ├── scene.xml
│   │   └── assets/
│   ├── franka_emika_panda/
│   ├── unitree_go1/
│   └── ...
├── scenes/
├── models/
└── ...
```

**备选方式 - 系统全局安装**：

```bash
# Windows
cd C:\Users\<YourName>\Documents
git clone https://github.com/google-deepmind/mujoco_menagerie.git

# Linux/Mac
cd ~/Projects
git clone https://github.com/google-deepmind/mujoco_menagerie.git
```

### 步骤 2：验证安装

检查关键模型是否存在：

```bash
# 检查 UR5e
ls menagerie/universal_robots_ur5e/

# 应该看到：
# ur5e.xml  scene.xml  assets/  LICENSE  README.md
```

---

## 集成方法

### 方法 1：使用 `<include>` 标签（推荐）

这是最简单且最易维护的方法。

#### 示例 1：集成 UR5e 机械臂

创建场景文件 `living_room_with_ur5e.xml`：

```xml
<mujoco model="living_room_with_ur5e">
  <compiler angle="degree" coordinate="local"/>

  <!-- 包含 UR5e 模型 -->
  <include file="../menagerie/universal_robots_ur5e/ur5e.xml"/>

  <worldbody>
    <!-- 场景元素 -->
    <geom name="floor" type="plane" size="5 5 0.1"/>

    <!-- 工作台 -->
    <body name="workbench" pos="0 0 0.5">
      <geom name="table" type="box" size="0.5 0.5 0.05"/>

      <!-- 放置 UR5e 机器人 -->
      <!-- UR5e 的根 body 名称是 "base_link" -->
      <body name="ur5e_mount" pos="0 0 0.05">
        <!-- 机器人会自动附加到这里 -->
      </body>
    </body>
  </worldbody>
</mujoco>
```

#### 示例 2：集成 Franka Panda

```xml
<mujoco model="scene_with_panda">
  <compiler angle="degree" coordinate="local"/>

  <!-- 包含 Franka Panda -->
  <include file="../menagerie/franka_emika_panda/panda.xml"/>

  <worldbody>
    <geom name="floor" type="plane" size="3 3 0.1"/>

    <!-- Panda 安装在桌面 -->
    <body name="table" pos="0 0 0.4">
      <geom type="box" size="0.4 0.4 0.02"/>

      <!-- Panda 的基座 -->
      <body name="panda_mount" pos="0 0 0.03">
        <!-- Panda 根 body: "link0" -->
      </body>
    </body>
  </worldbody>
</mujoco>
```

### 方法 2：直接使用 Menagerie 的 scene.xml

每个 Menagerie 模型都带有 `scene.xml`，可以直接运行：

```python
import mujoco
import mujoco.viewer

# 直接加载 Menagerie 的场景
model = mujoco.MjModel.from_xml_path(
    "menagerie/universal_robots_ur5e/scene.xml"
)
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
```

### 方法 3：复制并修改模型

如果需要高度定制：

1. 复制模型到项目 `models/` 目录
2. 修改 XML 文件
3. 在场景中引用

```bash
cp -r menagerie/universal_robots_ur5e models/robots/ur5e_custom
# 编辑 models/robots/ur5e_custom/ur5e.xml
```

---

## 常见机器人模型

### 工业机械臂

| 机器人 | Menagerie 路径 | 自由度 | 备注 |
|--------|----------------|--------|------|
| **UR5e** | `universal_robots_ur5e/ur5e.xml` | 6 DOF | 协作机器人，负载 5kg |
| **UR10e** | `universal_robots_ur10e/ur10e.xml` | 6 DOF | 更大工作空间，负载 10kg |
| **Franka Panda** | `franka_emika_panda/panda.xml` | 7 DOF | 冗余臂，适合研究 |
| **Kinova Gen3** | `kinova_gen3/gen3.xml` | 7 DOF | 轻量级协作臂 |

### 移动机器人

| 机器人 | Menagerie 路径 | 类型 | 备注 |
|--------|----------------|------|------|
| **Unitree Go1** | `unitree_go1/go1.xml` | 四足 | 敏捷四足机器人 |
| **ANYmal C** | `anybotics_anymal_c/anymal_c.xml` | 四足 | 重型四足平台 |
| **Boston Dynamics Spot** | `boston_dynamics_spot/spot.xml` | 四足 | 工业级四足机器人 |

### 人形与灵巧手

| 机器人 | Menagerie 路径 | 备注 |
|--------|----------------|------|
| **Unitree H1** | `unitree_h1/h1.xml` | 全尺寸人形机器人 |
| **Shadow Dexterous Hand** | `shadow_hand/shadow_hand.xml` | 24 DOF 灵巧手 |

---

## 完整集成示例

### 场景：客厅 + UR5e 机械臂

**文件**: `scenes/living_room_ur5e_complete.xml`

```xml
<mujoco model="living_room_ur5e_complete">
  <compiler angle="degree" coordinate="local" inertiafromgeom="true"/>

  <!-- 引入 UR5e -->
  <include file="../menagerie/universal_robots_ur5e/ur5e.xml"/>

  <option timestep="0.002" gravity="0 0 -9.81"/>

  <asset>
    <texture name="floor_tex" type="2d" builtin="checker"
             rgb1="0.8 0.8 0.8" rgb2="0.6 0.6 0.6"/>
    <material name="floor_mat" texture="floor_tex"/>
  </asset>

  <worldbody>
    <!-- 照明 -->
    <light pos="0 0 3" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>

    <!-- 地板 -->
    <geom name="floor" type="plane" size="3 3 0.1" material="floor_mat"/>

    <!-- 墙壁 -->
    <geom name="wall" type="box" pos="0 2 1" size="3 0.1 1"/>

    <!-- 工作台 -->
    <body name="workbench" pos="-1 -1 0.5">
      <geom name="table_top" type="box" size="0.6 0.6 0.05"
            rgba="0.6 0.4 0.2 1" mass="20"/>
      <geom name="leg1" type="cylinder" pos="0.5 0.5 -0.45"
            size="0.03 0.45" rgba="0.3 0.2 0.1 1"/>
      <geom name="leg2" type="cylinder" pos="0.5 -0.5 -0.45"
            size="0.03 0.45" rgba="0.3 0.2 0.1 1"/>
      <geom name="leg3" type="cylinder" pos="-0.5 0.5 -0.45"
            size="0.03 0.45" rgba="0.3 0.2 0.1 1"/>
      <geom name="leg4" type="cylinder" pos="-0.5 -0.5 -0.45"
            size="0.03 0.45" rgba="0.3 0.2 0.1 1"/>

      <!-- UR5e 安装 -->
      <body name="ur5e_base" pos="0 0 0.06" euler="0 0 0">
        <!--
        UR5e 的根 body 是 "base_link"
        它会自动附加到这个位置
        -->
      </body>
    </body>

    <!-- 可抓取物体 -->
    <body name="target_cube" pos="-0.8 -1.2 1.1">
      <freejoint/>
      <geom type="box" size="0.03 0.03 0.03"
            rgba="1 0 0 1" mass="0.1"/>
    </body>
  </worldbody>
</mujoco>
```

### Python 控制脚本

```python
#!/usr/bin/env python3
"""
UR5e 机械臂控制示例
"""
import mujoco
import mujoco.viewer
import numpy as np

# 加载场景
model = mujoco.MjModel.from_xml_path(
    "scenes/living_room_ur5e_complete.xml"
)
data = mujoco.MjData(model)

# 找到 UR5e 的执行器索引
print("执行器列表:")
for i in range(model.nu):
    actuator_name = mujoco.mj_id2name(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, i
    )
    print(f"  {i}: {actuator_name}")

# 设置目标关节位置（示例：home 姿态）
home_position = np.array([0, -1.57, 1.57, -1.57, -1.57, 0])

with mujoco.viewer.launch_passive(model, data) as viewer:
    # 设置相机
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -20
    viewer.cam.distance = 3.0

    step = 0
    while viewer.is_running():
        # 设置控制指令
        data.ctrl[:6] = home_position

        # 仿真步进
        mujoco.mj_step(model, data)

        # 每 100 步打印一次末端位置
        if step % 100 == 0:
            ee_pos = data.site('attachment_site').xpos
            print(f"Step {step}: EE position = {ee_pos}")

        viewer.sync()
        step += 1
```

---

## 路径配置

### 相对路径 vs 绝对路径

**相对路径（推荐）**：
```xml
<include file="../menagerie/universal_robots_ur5e/ur5e.xml"/>
```

**绝对路径**：
```xml
<!-- Windows -->
<include file="C:/Users/夜澜衣/Desktop/01/menagerie/universal_robots_ur5e/ur5e.xml"/>

<!-- Linux/Mac -->
<include file="/home/user/projects/01/menagerie/universal_robots_ur5e/ur5e.xml"/>
```

### 环境变量方式

设置环境变量：
```bash
# Windows (PowerShell)
$env:MUJOCO_MENAGERIE = "C:\Users\夜澜衣\Desktop\01\menagerie"

# Linux/Mac
export MUJOCO_MENAGERIE="/home/user/projects/01/menagerie"
```

在 XML 中使用：
```xml
<include file="$MUJOCO_MENAGERIE/universal_robots_ur5e/ur5e.xml"/>
```

---

## 常见问题

### Q1: 模型加载失败 - "File not found"

**原因**: 路径错误或 Menagerie 未克隆

**解决**：
```bash
# 检查路径
ls menagerie/universal_robots_ur5e/ur5e.xml

# 如果不存在，克隆 Menagerie
git clone https://github.com/google-deepmind/mujoco_menagerie.git menagerie
```

### Q2: 机器人位置不对

**原因**: 安装点 body 名称或位置不正确

**解决**：
- 检查 Menagerie 模型的根 body 名称
- 确保安装点位置合理（高度、旋转等）

```xml
<!-- 检查根 body -->
<body name="correct_mount_name" pos="0 0 0.5" euler="0 0 45">
```

### Q3: 执行器不工作

**原因**: Menagerie 模型的执行器名称与您的控制代码不匹配

**解决**：
```python
# 打印所有执行器名称
for i in range(model.nu):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    print(f"Actuator {i}: {name}")

# 按名称访问
actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "shoulder_pan")
data.ctrl[actuator_id] = 0.5
```

### Q4: 网格文件缺失

**原因**: Menagerie 模型依赖外部网格文件

**解决**：
- 确保整个 Menagerie 文件夹完整克隆（包括 `assets/` 子目录）
- 不要单独复制 XML 文件

---

## 高级技巧

### 1. 修改机器人外观

复制模型并修改材质：
```xml
<asset>
  <material name="custom_robot_color" rgba="0.8 0.2 0.2 1"
            specular="0.5" shininess="0.3"/>
</asset>

<!-- 在复制的模型中替换材质 -->
<geom ... material="custom_robot_color"/>
```

### 2. 添加末端执行器

在机器人末端附加夹爪：
```xml
<body name="ur5e_base" pos="0 0 0.5">
  <!-- UR5e body -->

  <!-- 在最后一个 link 后添加夹爪 -->
  <body name="gripper" pos="0 0 0.1">
    <geom name="finger_left" type="box" size="0.01 0.02 0.04"/>
    <geom name="finger_right" type="box" size="0.01 0.02 0.04"/>
  </body>
</body>
```

### 3. 批量导入多个机器人

```xml
<mujoco>
  <include file="../menagerie/universal_robots_ur5e/ur5e.xml"/>
  <include file="../menagerie/franka_emika_panda/panda.xml"/>

  <worldbody>
    <!-- UR5e 位置 1 -->
    <body name="ur5e_station_1" pos="-2 0 0.5"/>

    <!-- Panda 位置 2 -->
    <body name="panda_station_1" pos="2 0 0.5"/>
  </worldbody>
</mujoco>
```

---

## 下一步

1. ✅ 克隆 MuJoCo Menagerie 仓库
2. ✅ 尝试运行官方示例场景
3. ✅ 将机器人集成到您的自定义场景
4. ✅ 编写控制算法测试

**参考资源**：
- [MuJoCo Menagerie GitHub](https://github.com/google-deepmind/mujoco_menagerie)
- [MuJoCo 官方文档](https://mujoco.readthedocs.io/)
- [本项目机器人集成指南](robot_integration.md)

---

**最后更新**: 2026-01-15

# MuJoCo 儿童房机器人仿真项目 (Kid's Room Robot Simulation)

本项目是一个基于 MuJoCo 的高级家居场景仿真环境，专门设计用于机器人操纵任务（如抓取、整理玩具等）的数据收集与模型训练。

## 🚀 核心模型
- **终版模型文件**: [kids_room_mujoco.1.22.xml](file:///d:/01/scenes/kids_room_mujoco.1.22.xml)
- **场景描述**: 一个细节丰富的儿童房，包含床、柜子、书桌、各类玩具（如画笔、茶壶等）以及一个安装在基座上的 3 自由度机械臂（带夹爪）。

## 📸 数据收集与 LeRobot 格式
用户提到的“左边画面”即为录制的视频数据。在本项目中，我们通过 MuJoCo 脚本实现了类似 LeRobot 的数据收集流程。

### 原理说明
1. **视觉输入 (Visual Input)**: 
   - 在机械臂末端（[gripper_base](file:///d:/01/scenes/kids_room_mujoco.1.22.xml#L677)）添加了 `hand_camera`（眼在手上）。
   - 场景中还配置了 `top_view` 全景摄像头。
2. **状态记录 (State Logging)**:
   - 记录每一步的关节位置（`qpos`）和控制信号（`ctrl`）。
3. **数据打包**:
   - 将图像序列编码为视频（MP4），并将状态数据保存为结构化文件（如 `.npy` 或 `.hdf5`），这正是训练机器人学习模型（如模仿学习）所需的核心数据。

### 如何运行数据收集
我们提供了一个示例脚本 [record_lerobot_data.py](file:///d:/01/scripts/record_lerobot_data.py) 来演示这一过程：

```bash
# 安装依赖
pip install -r requirements.txt

# 运行录制脚本
python scripts/record_lerobot_data.py
```

运行后，脚本将在 `lerobot_data_sample` 目录下生成：
- `video.mp4`: 机械臂视角的录像。
- `states.npy`: 机械臂关节运动轨迹。
- `actions.npy`: 对应的控制指令。

## 🛠️ 项目结构
- `scenes/`: 存放 MuJoCo XML 场景文件。
- `models/`: 存放机器人（UR5, Humanoid）的简化模型。
- `scripts/`: 包含测试、验证及数据收集脚本。
- `assets/`: 存放 3D 网格模型（如 `teapot_simple.obj`）。

---
*注：本项目目前直接修改 MuJoCo 脚本进行数据收集，未来接入 OH (OpenHarmony) 或其他硬件平台时，底层驱动步骤会有所不同，但“状态+图像”的数据核心概念是一致的。*

# MuJoCo 仿真场景项目

> 高质量、物理准确的机器人仿真场景集合，专为算法测试与研究设计

![MuJoCo](https://img.shields.io/badge/MuJoCo-3.0+-blue.svg)
![Python](https://img.shields.io/badge/Python-3.8+-green.svg)
![License](https://img.shields.io/badge/License-MIT-yellow.svg)

## 项目目标

本项目旨在创建一套**美观、物理合理**的 MuJoCo 仿真场景，为机器人研究与算法开发提供高质量的测试环境。项目包含两大类场景：

- **家居环境**：客厅、厨房、卧室等日常生活场景，适用于服务机器人、移动操作等研究
- **工业环境**：仓库、工厂、物流中心等，适用于工业机器人、物流自动化等研究

### 核心特性

- ✅ **物理准确**：基于 MuJoCo 引擎的高精度物理仿真
- ✅ **视觉美观**：精心设计的材质、纹理和光照系统
- ✅ **模块化设计**：易于扩展和定制的场景结构
- ✅ **机器人兼容**：支持机械臂、人形机器人、移动机器人等多种类型
- ✅ **开箱即用**：提供完整的测试脚本和文档

---

## 项目结构

```
01/
├── scenes/              # 场景 XML 文件
│   ├── living_room_v2.xml     # 简化客厅场景
│   ├── home_complete.xml      # ⭐ 完整多房间家居场景
│   └── ...
├── models/              # 机器人模型文件
│   └── robots/          # 机器人 URDF/MJCF 文件（待添加）
├── assets/              # 资源文件
│   └── meshes/          # 3D 网格模型
│       └── teapot_simple.obj
├── scripts/             # 工具脚本
│   ├── test_scene.py         # 场景测试脚本
│   └── validate_scene.py     # 场景验证脚本
├── docs/                # 文档
│   └── scene_documentation.md  # home_complete 场景详细文档
└── README.md            # 本文件
```

---

## 快速开始

### 环境要求

- **Python**: 3.8 或更高版本
- **MuJoCo**: 3.0 或更高版本
- **依赖库**: `numpy`, `mujoco`

### 安装步骤

1. **克隆项目**
   ```bash
   git clone <repository_url>
   cd 01
   ```

2. **安装依赖**
   ```bash
   pip install mujoco numpy
   ```

3. **测试场景**
   ```bash
   python scripts/test_scene.py
   ```

### 运行第一个场景

```python
import mujoco
import mujoco.viewer

# 加载客厅场景
model = mujoco.MjModel.from_xml_path("scenes/living_room.xml")
data = mujoco.MjData(model)

# 启动可视化
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
```

---

## 场景介绍

### 1. 完整家居场景 (home_complete.xml) ⭐ 推荐

**描述**: 多房间完整家居环境，包含玄关、客厅、厨房、卧室和浴室

**特点**:
- 尺寸：14m × 12m × 2.5m（5个房间）
- **房间**: 玄关、客厅、厨房、卧室、浴室
- **可交互元素**:
  - 4个带物理铰链的门（可开关）
  - 2个电视柜门、2个厨房柜门、1个浴室柜门
  - 2个床头柜抽屉（可滑出）、1个马桶座圈、1个可开窗户
- **可移动物品**: 20+ 物品用于抓取任务
  - 客厅: 茶壶、书本、杯子、遥控器、靠垫
  - 厨房: 碗、盘子、杯子、水果（苹果、香蕉）
  - 卧室: 枕头、闹钟
  - 浴室: 牙刷架、香皂、洗发水、沐浴露、毛巾
- **光照**: 5个光源（主光+房间专属补光）
- **相机**: 7个预设视角（包含各房间视角）
- **任务标记**: 16+个机器人任务点（抓取、导航、操作）

**适用场景**:
- ✅ 服务机器人多房间导航
- ✅ 物体抓取与放置（pick-and-place）
- ✅ 门的开关操作
- ✅ 抽屉和柜门的开关
- ✅ 复杂的移动操作任务
- ✅ 人形机器人家务模拟
- ✅ 感知与场景理解

**使用方法**:
```bash
# 验证场景
python scripts/validate_scene.py home_complete.xml

# 启动可视化
python scripts/test_scene.py -s home_complete
```

**详细文档**: [场景文档](docs/scene_documentation.md)

---

### 2. 客厅场景 (living_room_v2.xml)

**描述**: 简化版客厅环境，适合快速测试

**特点**:
- 尺寸：6m × 6m × 2.5m
- 包含：地板（棋盘格纹理）、墙壁、桌子、沙发、地毯
- 光照：天花板主光源 + 环境补光
- 机器人位置：预留机器人生成点（绿色半透明平台）

**适用场景**:
- 服务机器人导航
- 物体抓取与操作
- 人机交互研究
- 人形机器人行走测试

**使用方法**:
```bash
python scripts/test_scene.py
```

---

## 机器人集成

项目支持多种机器人模型的无缝集成。

### 支持的机器人类型

| 类型 | 示例模型 | 状态 |
|------|----------|------|
| 机械臂 | UR5, Franka Panda, UR10 | 计划中 |
| 人形机器人 | Atlas, Cassie, Digit | 计划中 |
| 移动机器人 | TurtleBot, AMR | 计划中 |

### 集成方法

在场景 XML 中使用 `<include>` 标签引入机器人模型：

```xml
<mujoco model="living_room_with_robot">
  <include file="../models/robots/ur5/ur5.xml"/>

  <worldbody>
    <include file="living_room.xml"/>

    <!-- 将机器人放置在场景中 -->
    <body name="ur5_base" pos="-1.5 -1.5 0.8">
      <!-- 机器人基座定义 -->
    </body>
  </worldbody>
</mujoco>
```

详细教程请参见 [机器人集成指南](docs/robot_integration.md)（待创建）

---

## 开发路线图

### 第一阶段：家居场景 ✅ 已完成
- [x] 客厅基础场景（v1.0）
- [x] 多房间完整家居场景（v2.0 - home_complete.xml）
- [x] 浴室、卧室、厨房、玄关
- [x] 可交互的门、抽屉、柜门（14个关节）
- [x] 20+ 可移动物品用于操作任务
- [x] 窗户系统（4个窗户，1个可开关）
- [x] 完整的任务标记系统
- [ ] 添加机械臂集成示例
- [ ] 添加人形机器人集成示例

### 第二阶段：工业场景 🔄
- [ ] 仓库场景开发
- [ ] 货架和托盘模型
- [ ] AGV 小车集成
- [ ] 物流操作任务模板

### 第三阶段：高级特性 📋
- [ ] 动态光照系统
- [ ] 自定义纹理工具
- [ ] 场景编辑器 GUI
- [ ] 性能优化与基准测试

---

## 性能指南

### 渲染性能

- **推荐硬件**:
  - CPU: Intel i5 / AMD Ryzen 5 或更高
  - GPU: NVIDIA GTX 1060 或更高（支持 OpenGL 4.5+）
  - RAM: 8GB 或更多

- **优化建议**:
  - 调整 `<visual>` 中的 `shadowsize` 减少阴影开销
  - 使用 `offwidth` 和 `offheight` 控制渲染分辨率
  - 对于批量仿真，使用无头模式（headless rendering）

### 物理仿真性能

- 默认时间步长：`0.002s` (500 Hz)
- 实时因子：取决于场景复杂度
- 优化方法：减少不必要的碰撞检测（使用 `contype` 和 `conaffinity`）

---

## 贡献指南

欢迎贡献新场景、机器人模型或改进建议！

### 提交规范

1. Fork 本仓库
2. 创建特性分支：`git checkout -b feature/new-scene`
3. 提交更改：`git commit -m 'Add kitchen scene'`
4. 推送分支：`git push origin feature/new-scene`
5. 提交 Pull Request

### 场景设计规范

- 所有场景必须包含物理合理的碰撞设置
- 材质和纹理需有明确的注释说明
- 提供场景配套的测试脚本
- 更新 README 中的场景列表

---

## 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

---

## 联系与支持

- **Issues**: [GitHub Issues](https://github.com/your-username/mujoco-sim-scenes/issues)
- **Discussions**: [GitHub Discussions](https://github.com/your-username/mujoco-sim-scenes/discussions)

---

## 致谢

感谢以下项目和资源的启发：
- [MuJoCo](https://mujoco.org/) - Google DeepMind 开发的物理引擎
- [mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie) - 官方模型库
- [robosuite](https://github.com/ARISE-Initiative/robosuite) - 机器人操作环境

---

**最后更新**: 2026-01-16
**项目状态**: 🚀 第一阶段完成 - 功能完整的多房间家居场景已就绪

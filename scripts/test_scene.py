#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MuJoCo 场景测试脚本
用于加载和可视化场景文件，支持多个场景选择
确保场景能够正确渲染和运行
"""

import os
import sys
import argparse
import mujoco
import mujoco.viewer
import numpy as np

# 可用的场景列表
AVAILABLE_SCENES = {
    'living_room': 'living_room.xml',
    'living_room_v2': 'living_room_v2.xml',
    'living_room_ur5': 'living_room_with_ur5.xml',
    'living_room_humanoid': 'living_room_with_humanoid.xml',
    'living_room_menagerie': 'living_room_with_menagerie.xml',
    'home_complete': 'home_complete.xml',
}

def test_scene(scene_file):
    """
    加载并可视化指定的场景文件

    Args:
        scene_file: 场景文件名
    """
    # 获取场景文件路径
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    scene_path = os.path.join(project_root, "scenes", scene_file)

    print("=" * 60)
    print("MuJoCo 场景测试工具 v2.0")
    print("=" * 60)
    print(f"场景文件: {scene_file}")
    print(f"完整路径: {scene_path}")

    # 检查文件是否存在
    if not os.path.exists(scene_path):
        print(f"\n错误: 场景文件不存在！")
        print(f"请确保文件位于: {scene_path}")
        return False

    try:
        # 加载 MuJoCo 模型
        print("\n正在加载模型...")
        model = mujoco.MjModel.from_xml_path(scene_path)
        data = mujoco.MjData(model)

        print("✓ 模型加载成功！")
        print(f"  - 几何体数量: {model.ngeom}")
        print(f"  - Body 数量: {model.nbody}")
        print(f"  - 材质数量: {model.nmat}")
        print(f"  - 纹理数量: {model.ntex}")
        print(f"  - 光源数量: {model.nlight}")

        # 显示场景中的主要对象
        print("\n场景对象列表:")
        for i in range(model.nbody):
            body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
            if body_name:
                print(f"  - {body_name}")

        # 执行前向动力学（初始化）
        mujoco.mj_forward(model, data)

        print("\n✓ 物理仿真初始化成功！")
        print("\n正在启动可视化窗口...")
        print("提示:")
        print("  - 鼠标左键拖动: 旋转视角")
        print("  - 鼠标右键拖动: 平移视角")
        print("  - 鼠标滚轮: 缩放")
        print("  - 按 ESC 或关闭窗口退出")
        print("\n" + "=" * 60)

        # 启动交互式可视化
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # 设置相机初始位置（优化的默认视角）
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -20
            viewer.cam.distance = 7.0
            viewer.cam.lookat[:] = [0, 0, 0.8]

            # 运行仿真循环
            step_count = 0
            while viewer.is_running():
                # 推进仿真时间步
                mujoco.mj_step(model, data)

                # 更新可视化（每5步更新一次以提高流畅度）
                if step_count % 5 == 0:
                    viewer.sync()

                step_count += 1

        print("\n场景测试完成！")
        return True

    except Exception as e:
        print(f"\n错误: 加载场景失败！")
        print(f"错误信息: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

def list_scenes():
    """列出所有可用的场景"""
    print("\n可用场景列表:")
    print("=" * 60)
    for key, filename in AVAILABLE_SCENES.items():
        print(f"  {key:25s} -> {filename}")
    print("=" * 60)

def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description='MuJoCo 场景测试工具 - 加载并可视化仿真场景',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python test_scene.py                      # 测试默认场景 (living_room_v2)
  python test_scene.py -s living_room_ur5   # 测试 UR5 集成场景
  python test_scene.py -l                   # 列出所有可用场景
  python test_scene.py -f custom.xml        # 测试自定义场景文件
        """
    )

    parser.add_argument(
        '-s', '--scene',
        choices=list(AVAILABLE_SCENES.keys()),
        default='living_room_v2',
        help='选择要测试的场景 (默认: living_room_v2)'
    )

    parser.add_argument(
        '-f', '--file',
        type=str,
        help='直接指定场景文件名 (覆盖 -s 选项)'
    )

    parser.add_argument(
        '-l', '--list',
        action='store_true',
        help='列出所有可用的场景'
    )

    return parser.parse_args()

def check_dependencies():
    """
    检查必要的依赖库
    """
    print("检查依赖库...")
    dependencies = {
        'mujoco': 'MuJoCo Python 绑定',
        'numpy': 'NumPy 数值计算库'
    }

    missing = []
    for module, description in dependencies.items():
        try:
            __import__(module)
            print(f"  ✓ {description} ({module})")
        except ImportError:
            print(f"  ✗ {description} ({module}) - 未安装")
            missing.append(module)

    if missing:
        print(f"\n缺少依赖库: {', '.join(missing)}")
        print("请运行以下命令安装:")
        print(f"  pip install {' '.join(missing)}")
        return False

    print("✓ 所有依赖已安装\n")
    return True

if __name__ == "__main__":
    # 解析命令行参数
    args = parse_arguments()

    # 如果用户请求列出场景
    if args.list:
        list_scenes()
        exit(0)

    # 检查依赖
    if not check_dependencies():
        exit(1)

    # 确定要测试的场景文件
    if args.file:
        # 用户指定了自定义文件
        scene_file = args.file
        print(f"\n使用自定义场景文件: {scene_file}\n")
    else:
        # 使用预定义场景
        scene_file = AVAILABLE_SCENES[args.scene]
        print(f"\n使用场景: {args.scene} ({scene_file})\n")

    # 运行测试
    success = test_scene(scene_file)
    exit(0 if success else 1)

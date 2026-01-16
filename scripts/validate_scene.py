#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
快速验证 MuJoCo 场景文件语法
不需要启动可视化窗口
"""

import os
import sys

def validate_scene(scene_file):
    """
    验证场景文件是否可以被 MuJoCo 加载
    """
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    scene_path = os.path.join(project_root, "scenes", scene_file)

    print("=" * 70)
    print("MuJoCo 场景验证工具")
    print("=" * 70)
    print(f"场景文件: {scene_file}")
    print(f"完整路径: {scene_path}")

    # 检查文件是否存在
    if not os.path.exists(scene_path):
        print(f"\n[ERROR] File not found!")
        return False

    print(f"[OK] File exists")

    # 尝试导入 MuJoCo
    try:
        import mujoco
        print(f"[OK] MuJoCo installed (version: {mujoco.__version__})")
    except ImportError:
        print("\n[WARNING] MuJoCo not installed, performing basic XML validation only")

        # 尝试基本的 XML 解析
        try:
            import xml.etree.ElementTree as ET
            tree = ET.parse(scene_path)
            root = tree.getroot()
            print(f"[OK] XML syntax correct (root: {root.tag})")

            # 统计场景元素
            stats = {
                'asset': len(root.findall('.//asset/*')),
                'body': len(root.findall('.//body')),
                'geom': len(root.findall('.//geom')),
                'joint': len(root.findall('.//joint')),
                'light': len(root.findall('.//light')),
                'camera': len(root.findall('.//camera')),
                'actuator': len(root.findall('.//actuator/*')),
            }

            print("\nScene Statistics:")
            for key, count in stats.items():
                print(f"  - {key:12s}: {count:3d}")

            return True

        except Exception as e:
            print(f"\n[ERROR] XML parsing failed: {str(e)}")
            return False

    # 尝试加载 MuJoCo 模型
    try:
        print("\nLoading MuJoCo model...")
        model = mujoco.MjModel.from_xml_path(scene_path)
        data = mujoco.MjData(model)

        print("[OK] Model loaded successfully!\n")

        # 显示详细统计信息
        print("Scene Statistics:")
        stats = {
            "Geometries (geoms)": model.ngeom,
            "Bodies": model.nbody,
            "Joints": model.njnt,
            "Actuators": model.nu,
            "Materials": model.nmat,
            "Textures": model.ntex,
            "Lights": model.nlight,
            "Cameras": model.ncam,
            "Sites": model.nsite,
        }

        for name, count in stats.items():
            print(f"  - {name:25s}: {count:3d}")

        # 列出主要对象
        print("\nMain Objects:")
        body_count = 0
        for i in range(model.nbody):
            body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
            if body_name and body_name != 'world':
                body_count += 1
                if body_count <= 20:  # 只显示前20个
                    print(f"  {body_count:2d}. {body_name}")

        if body_count > 20:
            print(f"  ... Total {body_count} objects (showing first 20)")

        # 执行一步仿真验证物理引擎
        mujoco.mj_forward(model, data)
        print("\n[OK] Physics engine initialized")

        print("\n" + "=" * 70)
        print("[SUCCESS] Scene validation passed!")
        print("=" * 70)
        return True

    except Exception as e:
        print(f"\n[ERROR] Model loading failed!")
        print(f"Error message: {str(e)}")

        # 打印详细的错误堆栈
        import traceback
        print("\nDetailed error:")
        traceback.print_exc()

        return False

if __name__ == "__main__":
    # 如果提供了命令行参数，使用它；否则使用默认场景
    scene_file = sys.argv[1] if len(sys.argv) > 1 else "home_complete.xml"

    success = validate_scene(scene_file)

    if success:
        print("\nTo launch full visualization, use:")
        print(f"  python test_scene.py -f {scene_file}")

    exit(0 if success else 1)

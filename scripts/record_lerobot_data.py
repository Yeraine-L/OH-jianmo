import mujoco
import mujoco.viewer
import numpy as np
import imageio
import os
import time

# 数据收集脚本：模拟 LeRobot 数据格式收集
# 原理：
# 1. MuJoCo 仿真：负责物理模拟。
# 2. 摄像头渲染：通过 XML 中定义的 <camera> 标签获取图像。
# 3. 状态记录：在每一步仿真中，记录关节位置 (qpos) 和控制信号 (ctrl)。
# 4. 视频保存：将渲染出的帧序列保存为 MP4 文件。

# 场景路径
MODEL_PATH = os.path.join(os.path.dirname(__file__), "..", "scenes", "kids_room_mujoco.1.26.xml")
OUTPUT_DIR = "lerobot_data_sample_1.26"
os.makedirs(OUTPUT_DIR, exist_ok=True)

def record_demo():
    # 加载模型
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    
    # 渲染设置
    width, height = 640, 480
    renderer = mujoco.Renderer(model, height=height, width=width)
    
    # 视频记录器 (双摄像头：俯视 + 手部)
    video_writer_top = imageio.get_writer(os.path.join(OUTPUT_DIR, "video_top.mp4"), fps=30)
    video_writer_hand = imageio.get_writer(os.path.join(OUTPUT_DIR, "video_hand_1.mp4"), fps=30)
    
    # 数据存储列表
    states = []
    actions = []
    
    print(f"开始在 1.26 模型上收集数据 (包含 {model.nu} 个执行器)...")
    
    # 运行 300 步
    for i in range(300):
        # 简单的控制信号让机械臂 1 动起来
        # 机械臂 1 的执行器索引通常是 0-7
        data.ctrl[0] = np.sin(i / 20.0) * 1.0  # shoulder_pan
        data.ctrl[1] = np.cos(i / 20.0) * 0.5  # shoulder_lift
        data.ctrl[2] = np.sin(i / 15.0) * 0.8  # elbow
        data.ctrl[6] = (np.sin(i / 10.0) + 1) * 0.04 # gripper
        
        # 仿真步进
        mujoco.mj_step(model, data)
        
        # 每隔几步记录一次数据 (对应 30fps)
        step_skip = int(1 / (model.opt.timestep * 30))
        if step_skip == 0 or i % step_skip == 0:
            # 渲染俯视摄像头
            renderer.update_scene(data, camera="top_view")
            frame_top = renderer.render()
            video_writer_top.append_data(frame_top)
            
            # 渲染机械臂 1 手部摄像头
            renderer.update_scene(data, camera="hand_camera_1")
            frame_hand = renderer.render()
            video_writer_hand.append_data(frame_hand)
            
            # 记录所有关节状态和动作
            states.append(data.qpos.copy())
            actions.append(data.ctrl.copy())
            
            if i % 30 == 0:
                print(f"已录制 {i//30} 秒...")

    video_writer_top.close()
    video_writer_hand.close()
    
    # 保存数据
    np.save(os.path.join(OUTPUT_DIR, "states.npy"), np.array(states))
    np.save(os.path.join(OUTPUT_DIR, "actions.npy"), np.array(actions))
    
    print(f"数据收集完成！保存在 {OUTPUT_DIR} 目录下。")
    print("- video_top.mp4: 全景画面")
    print("- video_hand_1.mp4: 机械臂1视角")
    print("- states.npy: 完整关节状态")
    print("- actions.npy: 完整控制信号")

if __name__ == "__main__":
    record_demo()

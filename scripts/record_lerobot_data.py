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
MODEL_PATH = os.path.join(os.path.dirname(__file__), "..", "scenes", "kids_room_mujoco.1.22.xml")
OUTPUT_DIR = "lerobot_data_sample"
os.makedirs(OUTPUT_DIR, exist_ok=True)

def record_demo():
    # 加载模型
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    
    # 渲染设置
    width, height = 640, 480
    renderer = mujoco.Renderer(model, height=height, width=width)
    
    # 视频记录器
    video_writer = imageio.get_writer(os.path.join(OUTPUT_DIR, "video.mp4"), fps=30)
    
    # 数据存储列表
    states = []
    actions = []
    
    print("开始收集数据...")
    
    # 运行 300 步 (约 10 秒，假设 30fps)
    for i in range(300):
        # 简单的正弦控制信号让机械臂动起来
        data.ctrl[0] = np.sin(i / 20.0) * 1.5  # joint1
        data.ctrl[1] = np.cos(i / 20.0) * 0.5  # joint2
        data.ctrl[2] = np.sin(i / 10.0) * 0.5  # joint3
        
        # 仿真步进
        mujoco.mj_step(model, data)
        
        # 每隔几步记录一次数据 (对应 30fps)
        if i % (1 / (model.opt.timestep * 30)) < 1:
            # 渲染手部摄像头
            renderer.update_scene(data, camera="hand_camera")
            frame = renderer.render()
            video_writer.append_data(frame)
            
            # 记录状态和动作
            states.append(data.qpos[:5].copy())  # 记录前5个关节
            actions.append(data.ctrl[:5].copy())
            
            if i % 30 == 0:
                print(f"已录制 {i//30} 秒...")

    video_writer.close()
    
    # 保存状态数据为 numpy 文件 (模拟 LeRobot 的 hdf5/json 格式)
    np.save(os.path.join(OUTPUT_DIR, "states.npy"), np.array(states))
    np.save(os.path.join(OUTPUT_DIR, "actions.npy"), np.array(actions))
    
    print(f"数据收集完成！保存在 {OUTPUT_DIR} 目录下。")
    print("- video.mp4: 摄像头画面")
    print("- states.npy: 关节状态")
    print("- actions.npy: 控制信号")

if __name__ == "__main__":
    record_demo()

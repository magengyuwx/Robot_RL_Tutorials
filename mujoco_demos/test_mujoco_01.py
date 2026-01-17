
import mujoco
import mujoco.viewer
import time
import numpy as np

def mujoco_simple_test():
    # 1. 定义一个简单的单摆模型（XML格式）
    pendulum_xml = """
    <mujoco model="simple_pendulum">
        <compiler angle="radian" inertiafromgeom="true"/>
        <option timestep="0.005" gravity="0 0 -9.81"/>
        
        <worldbody>
            <!-- 地面 -->
            <geom name="ground" type="plane" pos="0 0 -1" size="5 5 0.1" rgba=".8 .8 .8 1"/>
            
            <!-- 摆杆的固定基座（父body） -->
            <body name="base" pos="0 0 1">
                <!-- 关节定义在base内部，连接base和pendulum -->
                <joint name="hinge" type="hinge" axis="0 1 0" damping="0.1"/>
                
                <!-- 摆杆本体（子body） -->
                <body name="pendulum">
                    <geom name="rod" type="capsule" fromto="0 0 0 0 0 -1" size="0.05" rgba=".2 .6 .8 1"/>
                    <geom name="bob" type="sphere" pos="0 0 -1" size="0.1" rgba=".8 .2 .2 1"/>
                    <inertial pos="0 0 -0.5" mass="1" diaginertia="0.1 0.1 0.1"/>
                </body>
            </body>
        </worldbody>
    </mujoco>
    """

    # 2. 加载模型和创建仿真数据结构
    model = mujoco.MjModel.from_xml_string(pendulum_xml)
    data = mujoco.MjData(model)

    # 3. 初始化可视化器并运行仿真
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 设置初始状态
        data.qpos[0] = np.pi / 4  # 摆杆初始角度（45度）
        
        # 仿真循环
        start_time = time.time()
        while True:
            # 计算仿真时间
            current_time = time.time() - start_time
            
            # 运行一步仿真
            mujoco.mj_step(model, data)
            
            # 同步可视化
            viewer.sync()
            
            # 控制仿真帧率（约60FPS）
            time.sleep(1/60)

if __name__ == "__main__":
    mujoco_simple_test()
from LQR import A, B, C, D, F_convergence,z
from leg_calculation import robot,Model,Data
import numpy as np
import mujoco as mj
from mujoco import viewer
import time

import socket
import struct

UDP_IP = "127.0.0.1"
UDP_PORT = 12345
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
vofa_tail = b'\x00\x00\x80\x7f'
v=0
x=0
Omega=0
with mj.viewer.launch_passive(Model, Data) as viewer:
    
    
    speed_factor = 1  # 1.0 表示实时速度，2.0 表示两倍速，0.5 表示半速，依此类推
    mj.mj_forward(Model, Data)
    start_wall_time = time.perf_counter() 
    while viewer.is_running():
        # 理想物理时间 = (当前现实经过的时间 * 倍速)
        target_sim_time = (time.perf_counter() - start_wall_time) * speed_factor
        
        # 2. 循环运行物理步，直到追上理想时间
        # 这样可以保证即便计算量大，物理时间也会尽量对齐现实
        while Data.time < target_sim_time:

            dt = Model.opt.timestep
            # 仿真主循环
            
            step_start = time.time()

            # --- 这里读取传感器数据 ---
            # 例如：angle = data.qpos[0]
            robot.read()
            robot.gimbal.angle+=robot.gimbal.gyro[1]*dt
            v+=robot.gimbal.accel[0]*dt
            x+=v*dt
            Omega+=robot.gimbal.gyro[2]*dt
            robot.forward()
            robot.backward()
            


            
            # --- 这里放置你的控制代码 ---
            # 例如：data.ctrl[0] = 1.0 
            elements = [x, Omega, 
            robot.leg_l.wheel.angle * robot.leg_l.wheel.longth,
            robot.leg_r.wheel.angle * robot.leg_r.wheel.longth,
            robot.gimbal.angle, 
            robot.leg_l.virtual_leg.angle, 
            robot.leg_r.virtual_leg.angle,
            v, 
            robot.gimbal.gyro[2], 
            robot.leg_l.wheel.vocity * robot.leg_l.wheel.longth,
            robot.leg_r.wheel.vocity * robot.leg_r.wheel.longth,
            robot.gimbal.gyro[1], 
            robot.leg_l.virtual_leg.vocity, 
            robot.leg_r.virtual_leg.vocity]
            #print("传感器数据：", elements)
            # 2. 转换为 numpy 数组并强制 reshape 为 (14, 1)
            # np.hstack 会处理掉那些 (1,) 形状的细微差别
            z = np.array([np.reshape(e, -1)[0] for e in elements]).reshape(14, 1)
            # print("状态向量 z :", z)
            #     [robot.gimbal.angle],[robot.leg_l.virtual_leg.angle],[robot.leg_r.virtual_leg.angle],
            #     [v],[robot.gimbal.gyro[2]],[robot.leg_l.wheel.vocity*robot.leg_l.wheel.longth],[robot.leg_r.wheel.vocity*robot.leg_r.wheel.longth],
            #     [robot.gimbal.gyro[1]],[robot.leg_l.virtual_leg.vocity],[robot.leg_r.virtual_leg.vocity]])
            U=-F_convergence @ z
            U=U.flatten()  # 将 (6, 1) 转换为 (6,) 的一维数组

            send_list = [
                x, 
                Omega, 
                robot.leg_l.wheel.angle * robot.leg_l.wheel.longth,
                robot.leg_r.wheel.angle * robot.leg_r.wheel.longth,
                robot.gimbal.angle, 
                robot.leg_l.virtual_leg.angle, 
                robot.leg_r.virtual_leg.angle,
                v, 
                robot.gimbal.gyro[2], 
                robot.leg_l.wheel.vocity * robot.leg_l.wheel.longth,
                robot.leg_r.wheel.vocity * robot.leg_r.wheel.longth,
                robot.gimbal.gyro[1], 
                robot.leg_l.virtual_leg.vocity, 
                robot.leg_r.virtual_leg.vocity,
                robot.leg_l.virtual_leg.longth, 
                robot.leg_r.virtual_leg.longth,
                U[0],
                U[1],
                U[2],
                U[3],
                U[4],
                U[5],

            ]
        

        # 3. 数据打包为二进制 (f 代表 float, < 代表小端)
        # 这里的 'ffff' 代表有 4 个 float 数据
            binary_data = struct.pack('<' + 'f' * len(send_list), *send_list)
            sock.sendto(binary_data + vofa_tail, (UDP_IP, UDP_PORT))
            a=2
            E=[a*(0.2-robot.leg_l.virtual_leg.longth),a*(robot.leg_r.virtual_leg.longth-0.2),a*(robot.leg_l.virtual_leg.longth-0.2),a*(0.2-robot.leg_r.virtual_leg.longth),0,0]
            Data.ctrl[:] = U
            # 3. 物理引擎向前推进一步
            mj.mj_step(Model, Data)

            
            # 3. 刷新渲染器 (这里通常对应显示器的刷新频率)
            viewer.sync()
            
            # 适当微休眠，防止 CPU 占用率 100%
            time.sleep(0.001)
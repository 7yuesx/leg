from LQR import F_convergence,z
from leg_calculation import robot,Model,Data
import numpy as np
import mujoco as mj
from mujoco import viewer
import time
from math import cos ,sin
import socket
import struct
from scipy.spatial.transform import Rotation as R

from user_class import PIDController,LowPassFilter

pid_leg_l_P1=PIDController(kp=0.1,ki=0,kd=0)
pid_leg_r_P1=PIDController(kp=0.1,ki=0,kd=0)
pid_leg_l_V1=PIDController(kp=0.01,ki=0,kd=0)
pid_leg_r_V1=PIDController(kp=0.01,ki=0,kd=0)
pid_leg_l_P2=PIDController(kp=0.1,ki=0,kd=0)
pid_leg_r_P2=PIDController(kp=0.1,ki=0,kd=0)
pid_leg_l_V2=PIDController(kp=0.01,ki=0,kd=0)
pid_leg_r_V2=PIDController(kp=0.01,ki=0,kd=0)




UDP_IP = "127.0.0.1"
UDP_PORT = 12345
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
vofa_tail = b'\x00\x00\x80\x7f'

leg_l_e=0
leg_r_e=0
leg_l_last=0
leg_r_last=0

v=0
x=0
Omega=0
roll=0
g_z=0


    

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

            #robot.gimbal.angle+=robot.gimbal.gyro[1]*dt
            #roll+=robot.gimbal.gyro[0]*dt
            #g_z=9.81*cos(robot.gimbal.angle)*cos(roll)
            # v+=(robot.gimbal.accel[0]+sin(robot.gimbal.angle)*9.81)*dt
            # x+=v*dt
            #Omega+=robot.gimbal.gyro[2]*dt



            # ... 仿真循环内 ...

            # 1. 读取传感器传回的四元数 [w, x, y, z]
            quat = robot.gimbal.quat

            # 2. Scipy 使用的顺序是 [x, y, z, w]，需要调整顺序
            quat_scipy = [quat[1], quat[2], quat[3], quat[0]]

            # 3. 转换为欧拉角 (以 xyz 顺规为例，返回弧度)
            # 如果你想看平衡车的倾角，通常关注的是 Pitch (y轴)
            euler = R.from_quat(quat_scipy).as_euler('xyz', degrees=True)

            roll, pitch, yaw = euler*3.1415926/180



            robot.forward()
            robot.leg_l.wheel.position_target=[0,0,-0.2]
            robot.leg_r.wheel.position_target=[0,0,-0.2]
            robot.backward()

            v=robot.gimbal.vocity[0]*cos(yaw)-robot.gimbal.vocity[1]*sin(yaw)
            x=robot.gimbal.position[0]*cos(yaw)-robot.gimbal.position[1]*sin(yaw)
            

            
            # --- 这里放置你的控制代码 ---
            # 例如：data.ctrl[0] = 1.0 
            elements = [x, yaw, 
            pitch, 
            robot.leg_l.virtual_leg.angle, 
            robot.leg_r.virtual_leg.angle,
            v, 
            robot.gimbal.gyro[2], 
            robot.gimbal.gyro[1], 
            robot.leg_l.virtual_leg.vocity, 
            robot.leg_r.virtual_leg.vocity]
            #print("传感器数据：", elements)
            # 2. 转换为 numpy 数组并强制 reshape 为 (10, 1)
            # np.hstack 会处理掉那些 (1,) 形状的细微差别
            z = np.array([np.reshape(e, -1)[0] for e in elements]).reshape(10, 1)
            # print("状态向量 z :", z)
            #     [robot.gimbal.angle],[robot.leg_l.virtual_leg.angle],[robot.leg_r.virtual_leg.angle],
            #     [v],[robot.gimbal.gyro[2]],[robot.leg_l.wheel.vocity*robot.leg_l.wheel.longth],[robot.leg_r.wheel.vocity*robot.leg_r.wheel.longth],
            #     [robot.gimbal.gyro[1]],[robot.leg_l.virtual_leg.vocity],[robot.leg_r.virtual_leg.vocity]])
            
            U=-F_convergence @ z
            #z = np.array([np.reshape(e, -1)[0] for e in elements]).reshape(1, 10)
            print(z)
            U=U.flatten()  # 将 (6, 1) 转换为 (6,) 的一维数组

         
            target_l_V1=float(pid_leg_l_P1.update(robot.leg_l.bigleg1.angle_target,robot.leg_l.bigleg1.angle))
         
            target_r_V1=float(pid_leg_l_P1.update(robot.leg_r.bigleg1.angle_target,robot.leg_r.bigleg1.angle))
         
            output_l1=float(pid_leg_l_V1.update(target_l_V1,robot.leg_l.bigleg1.vocity))
            output_r1=float(pid_leg_r_V1.update(target_r_V1,robot.leg_r.bigleg1.vocity))
            target_l_V2=float(pid_leg_l_P2.update(robot.leg_l.bigleg2.angle_target,robot.leg_l.bigleg2.angle))
       
            target_r_V2=float(pid_leg_l_P2.update(robot.leg_r.bigleg2.angle_target,robot.leg_r.bigleg2.angle))
  
            output_l2=float(pid_leg_l_V2.update(target_l_V2,robot.leg_l.bigleg2.vocity))
            output_r2=float(pid_leg_r_V2.update(target_r_V2,robot.leg_r.bigleg2.vocity))


            send_list = [
                z[0], 
                z[1], 
                z[2], 
                z[3], 
                z[4],
                z[5], 
                z[6], 
                z[7], 
                z[8], 
                z[9],
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


            pid=np.array([output_l1,output_r1,output_l2,output_r2,0,0])
            # Data.ctrl[:] = U+E*p+D*d
            U=np.clip(U,-20,20)
            Data.ctrl[:] = U
            
            # 3. 物理引擎向前推进一步

            mj.mj_step(Model, Data)
            
            # 3. 刷新渲染器 (这里通常对应显示器的刷新频率)
            viewer.sync()
            
            # 适当微休眠，防止 CPU 占用率 100%
        time.sleep(0.001)
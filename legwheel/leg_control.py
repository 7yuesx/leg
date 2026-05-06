from LQR import F_convergence, Ad, Bd, I 
import math
import numpy as np
import mujoco as mj
from mujoco import viewer
import time
from math import cos ,sin, pi
import socket
import struct
from scipy.spatial.transform import Rotation as R

from user_class import RobotSensor, Leg, State
from mymath import PID_control
from pynput import keyboard

# 控制状态
cmd_v = 0.0
cmd_omega_w = 0.0

cmd_hight = 0

def on_press(key):
    global cmd_v, cmd_omega_w, cmd_hight
    try:
        if key == keyboard.Key.up: cmd_v = 1.0
        if key == keyboard.Key.down: cmd_v = -1.0
        if key == keyboard.Key.left: cmd_omega_w = 1.0
        if key == keyboard.Key.right: cmd_omega_w = -1.0
        if key == keyboard.Key.shift_l:cmd_hight = 0.001
        if key == keyboard.Key.shift_r:cmd_hight = -0.001   
    except AttributeError:
        pass

def on_release(key):
    global cmd_v, cmd_omega_w, cmd_hight
    # 松开按键时归零
    cmd_v = 0.0
    cmd_omega_w = 0.0
    cmd_hight = 0

# 开启后台监听
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()


UDP_IP = "127.0.0.1"
UDP_PORT = 12345
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
vofa_tail = b'\x00\x00\x80\x7f'


    
model = mj.MjModel.from_xml_path("legwheel.xml")
data = mj.MjData(model)

dt = model.opt.timestep

sensor = RobotSensor(model, data)
robot_state = State()
left=Leg("left", dt)
right=Leg("right", dt)

left_length_pos = PID_control(kp=3000, ki=0.0, kd=2000, targ_value=0)
right_length_pos = PID_control(kp=3000, ki=0.0, kd=2000, targ_value=0)
left_length_vel = PID_control(kp=0, ki=0, kd=0, targ_value=0)
right_length_vel = PID_control(kp=0, ki=0, kd=0, targ_value=0)

x_target=0
w_target=0
x_dot_target=0
w_dot_target=0

leg_length=0.2

U_last = U = np.zeros((4,1))
U_out = np.zeros((6,1))
with mj.viewer.launch_passive(model, data) as viewer:
    
    while viewer.is_running():
        step_start = time.time()

        # 仿真主循环

        state = sensor.get_state()

        length = {
            "left_front_big": 0.0675*2,
            "left_back_big": 0.0675*2,
            "left_front_small": 0.1175*2,
            "left_back_small": 0.1175*2,
            "left_wheel":0.02,

            "right_front_big": 0.0675*2,
            "right_back_big": 0.0675*2,
            "right_front_small": 0.1175*2,
            "right_back_small": 0.1175*2,
            "right_wheel":0.02,
            
            "R":0.1625,
        }

        imu = {
            "acc": state["acc"],
            "gyro": state["gyro"],
            "euler": state["euler"]
        }

        motor = {
            "left_front_pos": state["joints"]["motor_l1_pos"],
            "left_back_pos": state["joints"]["motor_l2_pos"],
            "left_wheel_pos": state["joints"]["wheel_l_pos"],

            "right_front_pos": state["joints"]["motor_r1_pos"],
            "right_back_pos": state["joints"]["motor_r2_pos"],           
            "right_wheel_pos": state["joints"]["wheel_r_pos"],

            "left_front_vel": state["joints"]["motor_l1_vel"],
            "left_back_vel": state["joints"]["motor_l2_vel"],
            "left_wheel_vel": state["joints"]["wheel_l_vel"],

            "right_front_vel": state["joints"]["motor_r1_vel"],
            "right_back_vel": state["joints"]["motor_r2_vel"],
            "right_wheel_vel": state["joints"]["wheel_r_vel"],
        }    

        left.forward(motor, imu, length, robot_state.x[2,0])
        right.forward(motor, imu, length, robot_state.x[2,0])


        x_dot_target=cmd_v
        w_dot_target=cmd_omega_w
        # if x_dot_target>2:
        #     x_dot_target=2
        # if x_dot_target<-2:
        #     x_dot_target=-2
        # if w_dot_target>2:
        #     w_dot_target=2
        # if w_dot_target<-2:
        #     w_dot_target=-2
        x_target+=x_dot_target*dt
        w_target+=w_dot_target*dt


        U=U_last-F_convergence @ (np.block([[robot_state.update_state(left, right, length, motor,imu, dt)],
                            [robot_state.update_target(x_target, w_target, x_dot_target, w_dot_target)],
                            [robot_state.update_control(U_last)]]))

        # U_d=Bd.T @ (I-Ad)@robot_state.update_target(x_target, w_target, x_dot_target, w_dot_target)
        # U=U_d-F_convergence @ (np.block([[robot_state.update_state(left, right, length, motor,imu, dt)],
        #                     [robot_state.update_target(x_target, w_target, x_dot_target, w_dot_target)]]))

        # U[0,0]=0
        # U[1,0]=0
        # U[2,0]=0
        # U[3,0]=0

        leg_length+=cmd_hight

        # U=-F_convergence @ (robot_state.update_state(left, right, length, motor,imu, dt)-robot_state.update_target(x_target, w_target, x_dot_target, w_dot_target))
        left_length_vel.position_pid(0,left.length_dot.Diff(left.length))
        left_length_pos.position_pid(leg_length,left.length)
        right_length_vel.position_pid(0,right.length_dot.Diff(right.length))
        right_length_pos.position_pid(leg_length,right.length)

        max=100

        if left_length_pos.output>max:
            left_length_pos.output=max
        if left_length_pos.output<-max:
            left_length_pos.output=-max
        if right_length_pos.output>max:
            right_length_pos.output=max
        if right_length_pos.output<-max:
            right_length_pos.output=-max

        if left_length_vel.output>max:
            left_length_vel.output=max
        if left_length_vel.output<-max:
            left_length_vel.output=-max
        if right_length_vel.output>max:
            right_length_vel.output=max
        if right_length_vel.output<-max:
            right_length_vel.output=-max

        
        

        left.vmc(left_length_vel.output+left_length_pos.output+100*cos(robot_state.x[3,0]), U[0,0], motor, length)
        right.vmc(right_length_vel.output+right_length_pos.output+100*cos(robot_state.x[4,0]), U[1,0], motor, length)

        # left.vmc(0, U[0,0], motor, length)
        # right.vmc(0, U[1,0], motor, length)


        U_out = np.block([[left.t_front],[right.t_front],[left.t_back],[right.t_back],[U[2,0]],[U[3,0]]])
        # U_out = np.block([[left.t_front],[right.t_front],[left.t_back],[right.t_back],[U[2,0]],[U[3,0]]])
        g_chek=math.sqrt(imu["acc"][0]**2+imu["acc"][1]**2+imu["acc"][2]**2)
        if(g_chek<7.1):
            U_out[0,0]=0
            U_out[1,0]=0
            U_out[2,0]=0
            U_out[3,0]=0
            U_out[4,0]=0
            U_out[5,0]=0           
            
        send_list = [
            robot_state.x[0,0],
            robot_state.x[1,0],
            robot_state.x[2,0],
            robot_state.x[3,0],
            robot_state.x[4,0],
            robot_state.x[5,0],
            robot_state.x[6,0],
            robot_state.x[7,0],
            robot_state.x[8,0],
            robot_state.x[9,0],
            robot_state.x_target[0,0],
            robot_state.x_target[1,0],
            robot_state.x_target[2,0],
            robot_state.x_target[3,0],
            robot_state.x_target[4,0],
            robot_state.x_target[5,0],
            robot_state.x_target[6,0],
            robot_state.x_target[7,0],
            robot_state.x_target[8,0],
            robot_state.x_target[9,0],
            U[0,0],
            U[1,0],
            U_out[0,0],
            U_out[1,0],
            U_out[2,0],         
            U_out[3,0],
            U_out[4,0],
            U_out[5,0],
            imu["acc"][2],
            g_chek,
            left.length,
            right.length,
            left_length_pos.output,
            right_length_pos.output,
        ]
        

        # 3. 数据打包为二进制 (f 代表 float, < 代表小端)
        # 这里的 'ffff' 代表有 4 个 float 数据
        binary_data = struct.pack('<' + 'f' * len(send_list), *send_list)
        sock.sendto(binary_data + vofa_tail, (UDP_IP, UDP_PORT))

        if U_out[0,0]>15:
            U_out[0,0]=15
        if U_out[0,0]<-15:
            U_out[0,0]=-15
        if U_out[1,0]>15:
            U_out[1,0]=15
        if U_out[1,0]<-15:  
            U_out[1,0]=-15
        if U_out[2,0]>15:
            U_out[2,0]=15
        if U_out[2,0]<-15:
            U_out[2,0]=-15
        if U_out[3,0]>15:
            U_out[3,0]=15
        if U_out[3,0]<-15:
            U_out[3,0]=-15
        if U_out[4,0]>5:
            U_out[4,0]=5
        if U_out[4,0]<-5:
            U_out[4,0]=-5
        if U_out[5,0]>5:
            U_out[5,0]=5
        if U_out[5,0]<-5:
            U_out[5,0]=-5
        for i in range(6):
            data.ctrl[i] =  U_out[i,0]   
        # 3. 物理引擎向前推进一步

        mj.mj_step(model, data)
        
        # 3. 刷新渲染器 (这里通常对应显示器的刷新频率)
        viewer.sync()
        U_last=U
        # 适当微休眠，防止 CPU 占用率 100%
        time.sleep(dt*1)
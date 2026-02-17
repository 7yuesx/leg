from user_class import mj, Model, Data, robot
robot = robot()


gyro_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_SENSOR, "body_gyro")
accel_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_SENSOR, "body_accel")

pos1_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_SENSOR, "motor_l1_pos")
pos2_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_SENSOR, "motor_r1_pos")
pos3_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_SENSOR, "motor_l2_pos")
pos4_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_SENSOR, "motor_r2_pos")
pos5_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_SENSOR, "wheel_l_pos")
pos6_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_SENSOR, "wheel_r_pos")

vel1_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_SENSOR, "motor_l1_vel")
vel2_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_SENSOR, "motor_r1_vel")
vel3_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_SENSOR, "motor_l2_vel")
vel4_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_SENSOR, "motor_r2_vel")
vel5_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_SENSOR, "wheel_l_vel")
vel6_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_SENSOR, "wheel_r_vel")

force1_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_SENSOR, "motor_l1_force")
force2_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_SENSOR, "motor_r1_force")
force3_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_SENSOR, "motor_l2_force")
force4_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_SENSOR, "motor_r2_force")
force5_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_SENSOR, "wheel_l_force")
force6_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_SENSOR, "wheel_r_force")


bigleg = 0.135
smallleg = 0.235
r = 0.02
d = 0.05
robot.write(gyro_id, accel_id,
            pos1_id, pos2_id, pos3_id, pos4_id,pos5_id, pos6_id, 
            vel1_id, vel2_id, vel3_id, vel4_id, vel5_id, vel6_id,
            force1_id, force2_id, force3_id, force4_id, force5_id, force6_id,
            bigleg, smallleg, r,d)



   
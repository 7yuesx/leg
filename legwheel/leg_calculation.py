from user_class import mj, Model, Data, robot
robot = robot()



joint1_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_JOINT, "joint1")
joint2_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_JOINT, "joint2")
joint3_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_JOINT, "joint3")
joint4_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_JOINT, "joint4")
joint1_1_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_JOINT, "joint1_1")
joint2_1_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_JOINT, "joint2_1")
joint3_1_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_JOINT, "joint3_1")
joint4_1_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_JOINT, "joint4_1")
joint1__2_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_JOINT, "joint1__2")
joint2__2_id = mj.mj_name2id(Model, mj.mjtObj.mjOBJ_JOINT, "joint2__2")

joint1 =[0.05, 0.16, 0]
joint2 = [0.05, -0.16, 0]
joint3 = [-0.05, 0.16, 0]
joint4 = [-0.05, -0.16, 0]

joint1_1 = [0, 0.16, 0]
joint2_1 = [0, -0.16, 0]
joint3_1 = [0, 0.16, 0]
joint4_1 = [0, -0.16, 0]

joint1__2 = [0, 0.16, 0]
joint2__2 = [0, -0.16, 0]

bigleg = 0.135
smallleg = 0.235
r = 0.02
robot.write(joint1_id, joint2_id, joint3_id, joint4_id,
                    joint1_1_id, joint2_1_id, joint3_1_id, joint4_1_id, joint1__2_id, joint2__2_id, joint1, joint2, joint3, joint4,
                    joint1_1, joint2_1, joint3_1, joint4_1, joint1__2, joint2__2,
                    bigleg, smallleg, r)


while True:
    robot.read()
    robot.forward()
    robot.backward()
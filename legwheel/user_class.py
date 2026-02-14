import mujoco as mj
import math
from math import cos, sin, atan, atan2, sqrt, pi

Model = mj.MjModel.from_xml_path("legwheel.xml")
Data = mj.MjData(Model)



class model:
    def __init__(self, body=None, position=[None,None,None], position_target=[None,None,None], angle=None, angle_target=None, joint=None, longth=0):
        self.body_id = body
        self.position = position
        self.position_target = position_target
        self.angle = angle
        self.angle_target = angle_target
        self.joint_id = joint
        self.longth = longth

    def read(self):
        qpos_address = Model.jnt_qposadr[self.joint_id]
        self.angle = Data.qpos[qpos_address]
          

class virtual_body:
    def __init__(self, body_name=None, theta=None, longth=0):
        self.body_name = body_name
        self.theta = theta
        self.longth = longth

class gimbal:
    def __init__(self):
        self.gimbal = model()

class leg:
    def __init__(self):
        self.bigleg1 = model()
        self.bigleg2 = model()
        self.smallleg1 = model()
        self.smallleg2 = model()
        self.wheel = model()
        self.virtual_leg = virtual_body()
    def calculate(self):
        self.virtual_leg.theta = atan2(0-self.wheel.position[0],0-self.wheel.position[2])
        self.virtual_leg.longth = sqrt(self.wheel.position[2]**2 +self.wheel.position[0]**2)
class robot:
    def __init__(self):
        self.gimbal = gimbal()
        self.leg_l = leg()
        self.leg_r = leg()

    def read(self):
        self.leg_l.bigleg1.read()
        self.leg_l.bigleg2.read()
        self.leg_r.bigleg1.read()
        self.leg_r.bigleg2.read()

    def write(self,joint1_id, joint2_id, joint3_id, joint4_id,
                    joint1_1_id, joint2_1_id, joint3_1_id, joint4_1_id, joint1__2_id, joint2__2_id, joint1, joint2, joint3, joint4,
                    joint1_1, joint2_1, joint3_1, joint4_1, joint1__2, joint2__2,
                    bigleg, smallleg, r):
        
        self.leg_l.bigleg1.joint_id = joint1_id
        self.leg_l.bigleg2.joint_id = joint2_id
        self.leg_l.smallleg1.joint_id = joint1_1_id
        self.leg_l.smallleg2.joint_id = joint2_1_id
        self.leg_l.wheel.joint_id = joint1__2_id

        self.leg_r.bigleg1.joint_id = joint3_id
        self.leg_r.bigleg2.joint_id = joint4_id
        self.leg_r.smallleg1.joint_id = joint3_1_id
        self.leg_r.smallleg2.joint_id = joint4_1_id
        self.leg_r.wheel.joint_id = joint2__2_id

        self.leg_l.bigleg1.longth = bigleg
        self.leg_l.bigleg2.longth = bigleg
        self.leg_l.smallleg1.longth = smallleg
        self.leg_l.smallleg2.longth = smallleg
        self.leg_l.wheel.longth = r

        self.leg_r.bigleg1.longth = bigleg
        self.leg_r.bigleg2.longth = bigleg
        self.leg_r.smallleg1.longth = smallleg
        self.leg_r.smallleg2.longth = smallleg
        self.leg_r.wheel.longth = r

        self.leg_l.bigleg1.position = joint1
        self.leg_l.bigleg2.position = joint3
        self.leg_l.smallleg1.position = joint1_1
        self.leg_l.smallleg2.position = joint3_1
        self.leg_l.wheel.position = joint1__2

        self.leg_r.bigleg1.position = joint2
        self.leg_r.bigleg2.position = joint4
        self.leg_r.smallleg1.position = joint2_1
        self.leg_r.smallleg2.position = joint4_1
        self.leg_r.wheel.position = joint2__2


    def forward(self):

        self.leg_r.smallleg1.position[0] = self.leg_r.bigleg1.position[0] + self.leg_r.bigleg1.longth * cos(self.leg_r.bigleg1.angle)
        self.leg_r.smallleg1.position[2] = self.leg_r.bigleg1.position[2] + self.leg_r.bigleg1.longth * sin(self.leg_r.bigleg1.angle)

        self.leg_r.smallleg2.position[0] = self.leg_r.bigleg2.position[0] + self.leg_r.bigleg2.longth * cos(self.leg_r.bigleg2.angle)
        self.leg_r.smallleg2.position[2] = self.leg_r.bigleg2.position[2] + self.leg_r.bigleg2.longth * sin(self.leg_r.bigleg2.angle)

        p = self.leg_r.smallleg1.position[2]-self.leg_r.smallleg2.position[2]
        q = self.leg_r.smallleg1.position[0]-self.leg_r.smallleg2.position[0]
        a = self.leg_r.smallleg2.longth**2-self.leg_r.smallleg1.longth**2-p**2+q**2+2*q*self.leg_r.smallleg1.longth
        b = -4*p*self.leg_r.smallleg2.longth
        c = self.leg_r.smallleg2.longth**2-self.leg_r.smallleg1.longth**2-p**2-q**2-2*q*self.leg_r.smallleg1.longth


        z1 = 0.5 * (-b + sqrt(b**2-4*a*c))/a
        z2 = 0.5 * (-b - sqrt(b**2-4*a*c))/a

        if 2 * atan(z1) < -pi/2 :
                self.leg_r.smallleg1.angle = 2 * atan(z1);
        else:
                self.leg_r.smallleg1.angle = 2 * atan(z2);


        self.leg_r.wheel.position[0] = self.leg_r.smallleg1.position[0] + self.leg_r.smallleg1.longth * cos(self.leg_r.smallleg1.angle)
        self.leg_r.wheel.position[2] = self.leg_r.smallleg1.position[2] + self.leg_r.smallleg1.longth * sin(self.leg_r.smallleg1.angle)

        self.leg_r.calculate()
#############################################################################################################################################
        self.leg_l.smallleg1.position[0] = self.leg_l.bigleg1.position[0] + self.leg_l.bigleg1.longth * cos(self.leg_l.bigleg1.angle)
        self.leg_l.smallleg1.position[2] = self.leg_l.bigleg1.position[2] - self.leg_l.bigleg1.longth * sin(self.leg_l.bigleg1.angle)

        self.leg_l.smallleg2.position[0] = self.leg_l.bigleg2.position[0] + self.leg_l.bigleg2.longth * cos(self.leg_l.bigleg2.angle)
        self.leg_l.smallleg2.position[2] = self.leg_l.bigleg2.position[2] - self.leg_l.bigleg2.longth * sin(self.leg_l.bigleg2.angle)

        p = self.leg_l.smallleg1.position[2]-self.leg_l.smallleg2.position[2]
        q = self.leg_l.smallleg1.position[0]-self.leg_l.smallleg2.position[0]
        a = self.leg_l.smallleg2.longth**2-self.leg_l.smallleg1.longth**2-p**2+q**2+2*q*self.leg_l.smallleg1.longth
        b = 4*p*self.leg_l.smallleg2.longth
        c = self.leg_l.smallleg2.longth**2-self.leg_l.smallleg1.longth**2-p**2-q**2-2*q*self.leg_l.smallleg1.longth


        z1 = 0.5 * (-b + sqrt(b**2-4*a*c))/a
        z2 = 0.5 * (-b - sqrt(b**2-4*a*c))/a
        if 2 * atan(z1) > pi/2 :
                self.leg_l.smallleg1.angle = 2 * atan(z1);
        else:
                self.leg_l.smallleg1.angle = 2 * atan(z2);


        self.leg_l.wheel.position[0] = self.leg_l.smallleg1.position[0] + self.leg_l.smallleg1.longth * cos(self.leg_l.smallleg1.angle)
        self.leg_l.wheel.position[2] = self.leg_l.smallleg1.position[2] - self.leg_l.smallleg1.longth * sin(self.leg_l.smallleg1.angle)

        self.leg_l.calculate()


    def backward(self):
        p = self.leg_r.wheel.position_target[2]-self.leg_r.bigleg1.position[2]
        q = self.leg_r.wheel.position_target[0]-self.leg_r.bigleg1.position[0]

        a = q**2+p**2+self.leg_r.bigleg1.longth**2-self.leg_r.smallleg1.longth**2+2*self.leg_r.bigleg1.longth*self.leg_r.wheel.position_target[0]
        b = -4*self.leg_r.bigleg1.longth*self.leg_r.wheel.position_target[2]
        c = q**2+p**2+self.leg_r.bigleg1.longth**2-self.leg_r.smallleg1.longth**2-2*self.leg_r.bigleg1.longth*self.leg_r.wheel.position_target[0]
        z1 = 0.5 * (-b + sqrt(b**2-4*a*c))/a
        z2 = 0.5 * (-b - sqrt(b**2-4*a*c))/a

        if 2 * atan(z1) > -pi/3 and 2 * atan(z1) < pi/2:
                self.leg_r.bigleg1.angle_target = 2 * atan(z1);
        else:
                self.leg_r.bigleg1.angle_target = 2 * atan(z2);
        
        p = self.leg_r.wheel.position_target[2]-self.leg_r.bigleg2.position[2]
        q = self.leg_r.wheel.position_target[0]-self.leg_r.bigleg2.position[0]

        a = q**2+p**2+self.leg_r.bigleg2.longth**2-self.leg_r.smallleg1.longth**2+2*self.leg_r.bigleg2.longth*self.leg_r.wheel.position_target[0]
        b = -4*self.leg_r.bigleg2.longth*self.leg_r.wheel.position_target[2]
        c = q**2+p**2+self.leg_r.bigleg2.longth**2-self.leg_r.smallleg1.longth**2-2*self.leg_r.bigleg2.longth*self.leg_r.wheel.position_target[0]
        z1 = 0.5 * (-b + sqrt(b**2-4*a*c))/a
        z2 = 0.5 * (-b - sqrt(b**2-4*a*c))/a

        if 2 * atan(z1) > -5*pi/6 and 2 * atan(z1) < -pi/2:
                self.leg_r.bigleg2.angle_target = 2 * atan(z1);
        else:
                self.leg_r.bigleg2.angle_target = 2 * atan(z2);
######################################################################################################################################################
        p = self.leg_l.wheel.position_target[2]-self.leg_l.bigleg1.position[2]
        q = self.leg_l.wheel.position_target[0]-self.leg_l.bigleg1.position[0]

        a = q**2+p**2+self.leg_l.bigleg1.longth**2-self.leg_l.smallleg1.longth**2+2*self.leg_l.bigleg1.longth*self.leg_l.wheel.position_target[0]
        b = 4*self.leg_l.bigleg1.longth*self.leg_l.wheel.position_target[2]
        c = q**2+p**2+self.leg_l.bigleg1.longth**2-self.leg_l.smallleg1.longth**2-2*self.leg_l.bigleg1.longth*self.leg_l.wheel.position_target[0]
        z1 = 0.5 * (-b + sqrt(b**2-4*a*c))/a
        z2 = 0.5 * (-b - sqrt(b**2-4*a*c))/a

        if 2 * atan(z1) > -pi/3 and 2 * atan(z1) < pi/2:
                self.leg_l.bigleg1.angle_target = 2 * atan(z1);
        else:
                self.leg_l.bigleg1.angle_target = 2 * atan(z2);
        
        p = self.leg_l.wheel.position_target[2]-self.leg_l.bigleg2.position[2]
        q = self.leg_l.wheel.position_target[0]-self.leg_l.bigleg2.position[0]

        a = q**2+p**2+self.leg_l.bigleg2.longth**2-self.leg_l.smallleg1.longth**2+2*self.leg_l.bigleg2.longth*self.leg_l.wheel.position_target[0]
        b = 4*self.leg_l.bigleg2.longth*self.leg_l.wheel.position_target[2]
        c = q**2+p**2+self.leg_l.bigleg2.longth**2-self.leg_l.smallleg1.longth**2-2*self.leg_l.bigleg2.longth*self.leg_l.wheel.position_target[0]
        z1 = 0.5 * (-b + sqrt(b**2-4*a*c))/a
        z2 = 0.5 * (-b - sqrt(b**2-4*a*c))/a

        if 2 * atan(z1) > -5*pi/6 and 2 * atan(z1) < -pi/2:
                self.leg_l.bigleg2.angle_target = 2 * atan(z1);
        else:
                self.leg_l.bigleg2.angle_target = 2 * atan(z2);
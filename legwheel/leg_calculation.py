import math
from math import cos, sin, tan , atan2, sqrt, asin, acos

# class position:
#     def __init__(self, pos = [0, 0, 0]):
#         self.pos = pos
#     def distance(self, other):
#         return sum((a - b)**2 for a, b in zip(self.pos, other.pos))**0.5

# class angle:
#     def __init__(self, angle = 0):
#         self.angle = angle
#     def difference(self, other):
#         return (self.angle - other.angle)

# class body:
#     def __init__(self,longth):
#         self.longth = longth
class  int_robot:
    def __init__(self, joint1=None, joint2=None, joint3=None, joint4=None,
                joint1_1=None, joint2_1=None, joint3_1=None, joint4_1=None, joint2__2=None, joint4__2=None,
                gimbal_link=None, bigleg=None, smallleg=None, r=None,
                theta1=None, theta2=None, theta3=None, theta4=None,
                phi1=None, phi2=None, phi3=None, phi4=None):
        self.joint_bl1 = joint1
        self.joint_br1 = joint2
        self.joint_bl2 = joint3
        self.joint_br2 = joint4
        self.joint_sl1 = joint1_1
        self.joint_sr1 = joint2_1
        self.joint_sl2 = joint3_1
        self.joint_sr2 = joint4_1
        self.joint_w1 = joint2__2
        self.joint_w2 = joint4__2

        self.gimbal_link = gimbal_link
        self.bigleg1 = bigleg
        self.bigleg2 = bigleg
        self.bigleg3 = bigleg
        self.bigleg4 = bigleg
        self.smallleg1 = smallleg
        self.smallleg2 = smallleg
        self.smallleg3 = smallleg
        self.smallleg4 = smallleg
        self.wheel1 = r
        self.wheel2 = r

        self.theta1 = theta1
        self.theta2 = theta2
        self.theta3 = theta3
        self.theta4 = theta4
        self.phi1 = phi1
        self.phi2 = phi2
        self.phi3 = phi3
        self.phi4 = phi4


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

gimbal_link =   0.1
bigleg = 0.135

smallleg = 0.235

r = 0.02




legwheel = int_robot(joint1, joint2, joint3, joint4,
                    joint1_1, joint2_1, joint3_1, joint4_1, joint1__2, joint2__2,
                    gimbal_link, bigleg, smallleg, r)

joint1_1.pos[0] = joint1.pos[0] + bigleg1.longth * cos(theta1.angle)
joint1_1.pos[2] = joint1.pos[2] - bigleg1.longth * sin(theta1.angle)

joint3_1.pos[0] = joint3.pos[0] + bigleg3.longth * cos(theta3.angle)
joint3_1.pos[2] = joint3.pos[2] - bigleg3.longth * sin(theta3.angle)

def calculate(class int_robot):
    joint2_1.pos[0] = joint2.pos[0] + bigleg2.longth * cos(theta2.angle)
    joint2_1.pos[2] = joint2.pos[2] + bigleg2.longth * sin(theta2.angle)

    joint4_1.pos[0] = joint4.pos[0] + bigleg4.longth * cos(theta4.angle)
    joint4_1.pos[2] = joint4.pos[2] + bigleg4.longth * sin(theta4.angle)

    p = joint2_1.pos[2]-joint4_1.pos[2]
    q = joint2_1.pos[0]-joint4_1.pos[0]
    a = smallleg4.longth**2-smallleg2.longth**2-p**2-q**2+2*q*smallleg2.longth
    b = -4*p*smallleg2.longth
    c = smallleg4.longth**2-smallleg2.longth**2-p**2-q**2-2*q*smallleg2.longth


    z1 = 0.5 * (-b + sqrt(b**2-4*a*c))/a
    z2 = 0.5 * (-b - sqrt(b**2-4*a*c))/a
    angle1.angle = asin(2*z1/(1+z1**2))
    angle2.angle = asin(2*z2/(1+z2**2))
    if angle1.angle > -3.1415926 and angle1.angle < -3.1415926/2:
        phi2.angle = angle1.angle
    else:   phi2.angle = angle2.angle

    joint2__2.pos[0] = joint2_1.pos[0] + smallleg2.longth * cos(phi2.angle)
    joint2__2.pos[2] = joint2_1.pos[2] + smallleg2.longth * sin(phi2.angle)

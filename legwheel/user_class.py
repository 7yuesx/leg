import mujoco as mj
import math
from math import cos, sin, atan, atan2, sqrt, pi

Model = mj.MjModel.from_xml_path("legwheel.xml")
Data = mj.MjData(Model)




class gimbal:
    def __init__(self, body=None, position=[0,0,0], position_target=[0,0,0], angle=0, angle_target=0, vocity=0, force=0, gyro=[None,None,None], accel=[None,None,None], longth=0, sensor_gyro_id=None, sensor_accel_id=None):
        self.body_id = body
        self.position = position
        self.position_target = position_target
        self.angle = angle
        self.angle_target = angle_target
        self.vocity = vocity
        self.force = force
        self.gyro = gyro
        self.accel = accel
        self.longth = longth

        self.sensor_gyro_id = sensor_gyro_id
        self.sensor_accel_id = sensor_accel_id

    def read_gyro(self):
        start_index = Model.sensor_adr[self.sensor_gyro_id]
        data_dim = Model.sensor_dim[self.sensor_gyro_id]
        self.gyro = Data.sensordata[start_index : start_index + data_dim]
    def read_accel(self):
        start_index = Model.sensor_adr[self.sensor_accel_id]
        data_dim = Model.sensor_dim[self.sensor_accel_id]
        self.accel = Data.sensordata[start_index : start_index + data_dim]
          
class virtual_leg_l:
    def __init__(self, body_name=None, angle=0, longth=0, vocity=None):
        self.body_name = body_name
        self.angle = angle
        self.longth = longth
        self.vocity = vocity

class virtual_leg_r:
    def __init__(self, body_name=None, angle=0, longth=0, vocity=None):
        self.body_name = body_name
        self.angle = angle
        self.longth = longth
        self.vocity = vocity

class bigleg1_l:
    def __init__(self, body=None, position=[0,0,0], position_target=[0,0,0], angle=0, angle_target=0, vocity=0, force=0, gyro=[None,None,None], accel=[None,None,None], longth=0, sensor_pos_id=None, sensor_vel_id=None, sensor_force_id=None):
        self.body_id = body
        self.position = position
        self.position_target = position_target
        self.angle = angle
        self.angle_target = angle_target
        self.vocity = vocity
        self.force = force
        self.gyro = gyro
        self.accel = accel
        self.longth = longth
        self.sensor_pos_id = sensor_pos_id
        self.sensor_vel_id = sensor_vel_id
        self.sensor_force_id = sensor_force_id

    def read_pos(self):
        start_index = Model.sensor_adr[self.sensor_pos_id]
        data_dim = Model.sensor_dim[self.sensor_pos_id]
        self.angle = Data.sensordata[start_index : start_index + data_dim]
    def read_vel(self):
        start_index = Model.sensor_adr[self.sensor_vel_id]
        data_dim = Model.sensor_dim[self.sensor_vel_id]
        self.vocity = Data.sensordata[start_index : start_index + data_dim]
    def read_force(self):
        start_index = Model.sensor_adr[self.sensor_force_id]
        data_dim = Model.sensor_dim[self.sensor_force_id]
        self.force = Data.sensordata[start_index : start_index + data_dim]

        
class bigleg2_l:
    def __init__(self, body=None, position=[0,0,0], position_target=[0,0,0], angle=0, angle_target=0, vocity=0, force=0, gyro=[None,None,None], accel=[None,None,None], longth=0, sensor_pos_id=None, sensor_vel_id=None, sensor_force_id=None):
        self.body_id = body
        self.position = position
        self.position_target = position_target
        self.angle = angle
        self.angle_target = angle_target
        self.vocity = vocity
        self.force = force
        self.gyro = gyro
        self.accel = accel
        self.longth = longth
        self.sensor_pos_id = sensor_pos_id
        self.sensor_vel_id = sensor_vel_id
        self.sensor_force_id = sensor_force_id

    def read_pos(self):
        start_index = Model.sensor_adr[self.sensor_pos_id]
        data_dim = Model.sensor_dim[self.sensor_pos_id]
        self.angle = Data.sensordata[start_index : start_index + data_dim]
    def read_vel(self):
        start_index = Model.sensor_adr[self.sensor_vel_id]
        data_dim = Model.sensor_dim[self.sensor_vel_id]
        self.vocity = Data.sensordata[start_index : start_index + data_dim]
    def read_force(self):
        start_index = Model.sensor_adr[self.sensor_force_id]
        data_dim = Model.sensor_dim[self.sensor_force_id]
        self.force = Data.sensordata[start_index : start_index + data_dim]
          
class smallleg1_l:
    def __init__(self, body=None, position=[0,0,0], position_target=[0,0,0], angle=0, angle_target=0, vocity=0, force=0, gyro=[None,None,None], accel=[None,None,None], longth=0):
        self.body_id = body
        self.position = position
        self.position_target = position_target
        self.angle = angle
        self.angle_target = angle_target
        self.vocity = vocity
        self.force = force
        self.gyro = gyro
        self.accel = accel
        self.longth = longth
         
class smallleg2_l:
    def __init__(self, body=None, position=[0,0,0], position_target=[0,0,0], angle=0, angle_target=0, vocity=0, force=0, gyro=[None,None,None], accel=[None,None,None], longth=0):
        self.body_id = body
        self.position = position
        self.position_target = position_target
        self.angle = angle
        self.angle_target = angle_target
        self.vocity = vocity
        self.force = force
        self.gyro = gyro
        self.accel = accel
        self.longth = longth


class wheel_l:
    def __init__(self, body=None, position=[0,0,0], position_target=[0,0,-0.2], angle=0, angle_target=0, vocity=0, force=0, gyro=[None,None,None], accel=[None,None,None], longth=0, sensor_pos_id=None, sensor_vel_id=None, sensor_force_id=None):
        self.body_id = body
        self.position = position
        self.position_target = position_target
        self.angle = angle
        self.angle_target = angle_target
        self.vocity = vocity
        self.force = force
        self.gyro = gyro
        self.accel = accel
        self.longth = longth
        self.sensor_pos_id = sensor_pos_id
        self.sensor_vel_id = sensor_vel_id
        self.sensor_force_id = sensor_force_id

    def read_pos(self):
        start_index = Model.sensor_adr[self.sensor_pos_id]
        data_dim = Model.sensor_dim[self.sensor_pos_id]
        self.angle = Data.sensordata[start_index : start_index + data_dim]
    def read_vel(self):
        start_index = Model.sensor_adr[self.sensor_vel_id]
        data_dim = Model.sensor_dim[self.sensor_vel_id]
        self.vocity = Data.sensordata[start_index : start_index + data_dim]
    def read_force(self):
        start_index = Model.sensor_adr[self.sensor_force_id]
        data_dim = Model.sensor_dim[self.sensor_force_id]
        self.force = Data.sensordata[start_index : start_index + data_dim]

class bigleg1_r:
    def __init__(self, body=None, position=[0,0,0], position_target=[0,0,0], angle=0, angle_target=0, vocity=0, force=0, gyro=[None,None,None], accel=[None,None,None], longth=0, sensor_pos_id=None, sensor_vel_id=None, sensor_force_id=None):
        self.body_id = body
        self.position = position
        self.position_target = position_target
        self.angle = angle
        self.angle_target = angle_target
        self.vocity = vocity
        self.force = force
        self.gyro = gyro
        self.accel = accel
        self.longth = longth
        self.sensor_pos_id = sensor_pos_id
        self.sensor_vel_id = sensor_vel_id
        self.sensor_force_id = sensor_force_id

    def read_pos(self):
        start_index = Model.sensor_adr[self.sensor_pos_id]
        data_dim = Model.sensor_dim[self.sensor_pos_id]
        self.angle = Data.sensordata[start_index : start_index + data_dim]
    def read_vel(self):
        start_index = Model.sensor_adr[self.sensor_vel_id]
        data_dim = Model.sensor_dim[self.sensor_vel_id]
        self.vocity = Data.sensordata[start_index : start_index + data_dim]
    def read_force(self):
        start_index = Model.sensor_adr[self.sensor_force_id]
        data_dim = Model.sensor_dim[self.sensor_force_id]
        self.force = Data.sensordata[start_index : start_index + data_dim]

        
class bigleg2_r:
    def __init__(self, body=None, position=[0,0,0], position_target=[0,0,0], angle=0, angle_target=0, vocity=0, force=0, gyro=[None,None,None], accel=[None,None,None], longth=0, sensor_pos_id=None, sensor_vel_id=None, sensor_force_id=None,):
        self.body_id = body
        self.position = position
        self.position_target = position_target
        self.angle = angle
        self.angle_target = angle_target
        self.vocity = vocity
        self.force = force
        self.gyro = gyro
        self.accel = accel
        self.longth = longth
        self.sensor_pos_id = sensor_pos_id
        self.sensor_vel_id = sensor_vel_id
        self.sensor_force_id = sensor_force_id

    def read_pos(self):
        start_index = Model.sensor_adr[self.sensor_pos_id]
        data_dim = Model.sensor_dim[self.sensor_pos_id]
        self.angle = Data.sensordata[start_index : start_index + data_dim]
    def read_vel(self):
        start_index = Model.sensor_adr[self.sensor_vel_id]
        data_dim = Model.sensor_dim[self.sensor_vel_id]
        self.vocity = Data.sensordata[start_index : start_index + data_dim]
    def read_force(self):
        start_index = Model.sensor_adr[self.sensor_force_id]
        data_dim = Model.sensor_dim[self.sensor_force_id]
        self.force = Data.sensordata[start_index : start_index + data_dim]

          
class smallleg1_r:
    def __init__(self, body=None, position=[0,0,0], position_target=[0,0,0], angle=0, angle_target=0, vocity=0, force=0, gyro=[None,None,None], accel=[None,None,None], longth=0):
        self.body_id = body
        self.position = position
        self.position_target = position_target
        self.angle = angle
        self.angle_target = angle_target
        self.vocity = vocity
        self.force = force
        self.gyro = gyro
        self.accel = accel
        self.longth = longth
        
         
class smallleg2_r:
    def __init__(self, body=None, position=[0,0,0], position_target=[0,0,0], angle=0, angle_target=0, vocity=0, force=0, gyro=[None,None,None], accel=[None,None,None], longth=0):
        self.body_id = body
        self.position = position
        self.position_target = position_target
        self.angle = angle
        self.angle_target = angle_target
        self.vocity = vocity
        self.force = force
        self.gyro = gyro
        self.accel = accel
        self.longth = longth

class wheel_r:
    def __init__(self, body=None, position=[0,0,0], position_target=[0,0,-0.2], angle=0, angle_target=0, vocity=0, force=0, gyro=[None,None,None], accel=[None,None,None], longth=0, sensor_pos_id=None, sensor_vel_id=None, sensor_force_id=None):
        self.body_id = body
        self.position = position
        self.position_target = position_target
        self.angle = angle
        self.angle_target = angle_target
        self.vocity = vocity
        self.force = force
        self.gyro = gyro
        self.accel = accel
        self.longth = longth
        self.sensor_pos_id = sensor_pos_id
        self.sensor_vel_id = sensor_vel_id
        self.sensor_force_id = sensor_force_id

    def read_pos(self):
        start_index = Model.sensor_adr[self.sensor_pos_id]
        data_dim = Model.sensor_dim[self.sensor_pos_id]
        self.angle = Data.sensordata[start_index : start_index + data_dim]
    def read_vel(self):
        start_index = Model.sensor_adr[self.sensor_vel_id]
        data_dim = Model.sensor_dim[self.sensor_vel_id]
        self.vocity = Data.sensordata[start_index : start_index + data_dim]
    def read_force(self):
        start_index = Model.sensor_adr[self.sensor_force_id]
        data_dim = Model.sensor_dim[self.sensor_force_id]
        self.force = Data.sensordata[start_index : start_index + data_dim]



class leg_l:
    def __init__(self):
        self.bigleg1 = bigleg1_l()
        self.bigleg2 = bigleg2_l()
        self.smallleg1 = smallleg1_l()
        self.smallleg2 = smallleg2_l()
        self.wheel = wheel_l()
        self.virtual_leg = virtual_leg_l()
    def calculate(self):
        self.virtual_leg.angle = atan2(self.wheel.position[0]-0,self.wheel.position[2]-0)
        self.virtual_leg.longth = sqrt(self.wheel.position[2]**2 +self.wheel.position[0]**2)
        self.virtual_leg.vocity = (-self.bigleg1.vocity)*self.bigleg1.longth/self.virtual_leg.longth*cos(self.smallleg1.angle+self.virtual_leg.angle)*sin(self.smallleg1.angle-self.bigleg1.angle)
        +(-self.bigleg2.vocity)*self.bigleg2.longth/self.virtual_leg.longth*cos(self.smallleg2.angle+self.virtual_leg.angle)*sin(self.smallleg2.angle-self.bigleg2.angle)

class leg_r:
    def __init__(self):
        self.bigleg1 = bigleg1_r()
        self.bigleg2 = bigleg2_r()
        self.smallleg1 = smallleg1_r()
        self.smallleg2 = smallleg2_r()
        self.wheel = wheel_r()
        self.virtual_leg = virtual_leg_r()
    def calculate(self):
        self.virtual_leg.angle = atan2(self.wheel.position[0]-0,self.wheel.position[2]-0)
        self.virtual_leg.longth = sqrt(self.wheel.position[2]**2 +self.wheel.position[0]**2)
        self.virtual_leg.vocity = self.bigleg1.vocity*self.bigleg1.longth/self.virtual_leg.longth*cos(self.smallleg1.angle+self.virtual_leg.angle)*sin(self.smallleg1.angle-(-self.bigleg1.angle))
        +self.bigleg2.vocity*self.bigleg2.longth/self.virtual_leg.longth*cos(self.smallleg2.angle+self.virtual_leg.angle)*sin(self.smallleg2.angle-(-self.bigleg2.angle))


class robot:
    def __init__(self):
        self.gimbal = gimbal()
        self.leg_l = leg_l()
        self.leg_r = leg_r()

    def read(self):
        self.gimbal.read_gyro()
        self.gimbal.read_accel()
        self.leg_l.bigleg1.read_pos()
        self.leg_l.bigleg2.read_pos()
        self.leg_r.bigleg1.read_pos()
        self.leg_r.bigleg2.read_pos()
        self.leg_l.wheel.read_pos()
        self.leg_r.wheel.read_pos()
        self.leg_l.bigleg1.read_vel()
        self.leg_l.bigleg2.read_vel()
        self.leg_r.bigleg1.read_vel()
        self.leg_r.bigleg2.read_vel()
        self.leg_l.wheel.read_vel()
        self.leg_r.wheel.read_vel()

    def write(self,gyro_id, accel_id,
                pos1_id, pos2_id, pos3_id, pos4_id,pos5_id, pos6_id, 
                vel1_id, vel2_id, vel3_id, vel4_id, vel5_id, vel6_id,
                force1_id, force2_id, force3_id, force4_id, force5_id, force6_id,
                bigleg, smallleg, r,d):
        
        self.gimbal.sensor_gyro_id = gyro_id
        self.gimbal.sensor_accel_id = accel_id
        self.leg_l.bigleg1.sensor_pos_id = pos1_id
        self.leg_l.bigleg2.sensor_pos_id = pos3_id
        self.leg_l.wheel.sensor_pos_id = pos5_id

        self.leg_r.bigleg1.sensor_pos_id = pos2_id
        self.leg_r.bigleg2.sensor_pos_id = pos4_id
        self.leg_r.wheel.sensor_pos_id = pos6_id
        
        self.leg_l.bigleg1.sensor_vel_id = vel1_id
        self.leg_l.bigleg2.sensor_vel_id = vel3_id
        self.leg_l.wheel.sensor_vel_id = vel5_id

        self.leg_r.bigleg1.sensor_vel_id = vel2_id
        self.leg_r.bigleg2.sensor_vel_id = vel4_id
        self.leg_r.wheel.sensor_vel_id = vel6_id

        self.leg_l.bigleg1.sensor_force_id = force1_id
        self.leg_l.bigleg2.sensor_force_id = force3_id
        self.leg_l.wheel.sensor_force_id = force5_id

        self.leg_r.bigleg1.sensor_force_id = force2_id
        self.leg_r.bigleg2.sensor_force_id = force4_id
        self.leg_r.wheel.sensor_force_id = force6_id

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

        self.gimbal.longth = d

    def forward(self):
        
        self.leg_r.bigleg1.position[0]=self.gimbal.longth*cos(self.gimbal.angle)
        self.leg_r.bigleg1.position[2]=self.gimbal.longth*sin(self.gimbal.angle)

        self.leg_r.bigleg2.position[0]=-self.gimbal.longth*cos(self.gimbal.angle)
        self.leg_r.bigleg2.position[2]=-self.gimbal.longth*sin(self.gimbal.angle)

        self.leg_r.smallleg1.position[0] = self.leg_r.bigleg1.position[0] + self.leg_r.bigleg1.longth * cos(-self.leg_r.bigleg1.angle)
        self.leg_r.smallleg1.position[2] = self.leg_r.bigleg1.position[2] + self.leg_r.bigleg1.longth * sin(-self.leg_r.bigleg1.angle)

        self.leg_r.smallleg2.position[0] = self.leg_r.bigleg2.position[0] + self.leg_r.bigleg2.longth * cos(-self.leg_r.bigleg2.angle)
        self.leg_r.smallleg2.position[2] = self.leg_r.bigleg2.position[2] + self.leg_r.bigleg2.longth * sin(-self.leg_r.bigleg2.angle)

        p = self.leg_r.smallleg1.position[2]-self.leg_r.smallleg2.position[2]
        q = self.leg_r.smallleg1.position[0]-self.leg_r.smallleg2.position[0]
        a = self.leg_r.smallleg2.longth**2-self.leg_r.smallleg1.longth**2-p**2-q**2+2*q*self.leg_r.smallleg1.longth
        b = -4*p*self.leg_r.smallleg2.longth
        c = self.leg_r.smallleg2.longth**2-self.leg_r.smallleg1.longth**2-p**2-q**2-2*q*self.leg_r.smallleg1.longth


        z1 = 0.5 * (-b + sqrt(b**2-4*a*c))/a
        z2 = 0.5 * (-b - sqrt(b**2-4*a*c))/a

        if 2 * atan(z1) > pi/2 :
                self.leg_r.smallleg1.angle = 2 * atan(z1)
        else:
                self.leg_r.smallleg1.angle = 2 * atan(z2)


        self.leg_r.wheel.position[0] = self.leg_r.smallleg1.position[0] + self.leg_r.smallleg1.longth * cos(self.leg_r.smallleg1.angle)
        self.leg_r.wheel.position[2] = self.leg_r.smallleg1.position[2] + self.leg_r.smallleg1.longth * sin(self.leg_r.smallleg1.angle)
#####################
        p = -(self.leg_r.smallleg1.position[2]-self.leg_r.smallleg2.position[2])
        q = -(self.leg_r.smallleg1.position[0]-self.leg_r.smallleg2.position[0])
        a = self.leg_r.smallleg2.longth**2-self.leg_r.smallleg1.longth**2-p**2-q**2+2*q*self.leg_r.smallleg1.longth
        b = -4*p*self.leg_r.smallleg2.longth
        c = self.leg_r.smallleg2.longth**2-self.leg_r.smallleg1.longth**2-p**2-q**2-2*q*self.leg_r.smallleg1.longth


        z1 = 0.5 * (-b + sqrt(b**2-4*a*c))/a
        z2 = 0.5 * (-b - sqrt(b**2-4*a*c))/a

        if 2 * atan(z1) > pi/2 :
                self.leg_r.smallleg2.angle = 2 * atan(z1)
        else:
                self.leg_r.smallleg2.angle = 2 * atan(z2)

        self.leg_r.calculate()
#############################################################################################################################################
        self.leg_l.bigleg1.position[0]=self.gimbal.longth*cos(self.gimbal.angle)
        self.leg_l.bigleg1.position[2]=self.gimbal.longth*sin(self.gimbal.angle)

        self.leg_l.bigleg2.position[0]=-self.gimbal.longth*cos(self.gimbal.angle)
        self.leg_l.bigleg2.position[2]=-self.gimbal.longth*sin(self.gimbal.angle)

        self.leg_l.smallleg1.position[0] = self.leg_l.bigleg1.position[0] + self.leg_l.bigleg1.longth * cos(self.leg_l.bigleg1.angle)
        self.leg_l.smallleg1.position[2] = self.leg_l.bigleg1.position[2] + self.leg_l.bigleg1.longth * sin(self.leg_l.bigleg1.angle)

        self.leg_l.smallleg2.position[0] = self.leg_l.bigleg2.position[0] + self.leg_l.bigleg2.longth * cos(self.leg_l.bigleg2.angle)
        self.leg_l.smallleg2.position[2] = self.leg_l.bigleg2.position[2] + self.leg_l.bigleg2.longth * sin(self.leg_l.bigleg2.angle)

        p = self.leg_l.smallleg1.position[2]-self.leg_l.smallleg2.position[2]
        q = self.leg_l.smallleg1.position[0]-self.leg_l.smallleg2.position[0]
        a = self.leg_l.smallleg2.longth**2-self.leg_l.smallleg1.longth**2-p**2-q**2+2*q*self.leg_l.smallleg1.longth
        b = -4*p*self.leg_l.smallleg2.longth
        c = self.leg_l.smallleg2.longth**2-self.leg_l.smallleg1.longth**2-p**2-q**2-2*q*self.leg_l.smallleg1.longth


        z1 = 0.5 * (-b + sqrt(b**2-4*a*c))/a
        z2 = 0.5 * (-b - sqrt(b**2-4*a*c))/a

        if 2 * atan(z1) > pi/2 :
                self.leg_l.smallleg1.angle = 2 * atan(z1)
        else:
                self.leg_l.smallleg1.angle = 2 * atan(z2)


        self.leg_l.wheel.position[0] = self.leg_l.smallleg1.position[0] + self.leg_l.smallleg1.longth * cos(self.leg_l.smallleg1.angle)
        self.leg_l.wheel.position[2] = self.leg_l.smallleg1.position[2] + self.leg_l.smallleg1.longth * sin(self.leg_l.smallleg1.angle)
####################################
        p = -(self.leg_l.smallleg1.position[2]-self.leg_l.smallleg2.position[2])
        q = -(self.leg_l.smallleg1.position[0]-self.leg_l.smallleg2.position[0])
        a = self.leg_l.smallleg2.longth**2-self.leg_l.smallleg1.longth**2-p**2-q**2+2*q*self.leg_l.smallleg1.longth
        b = -4*p*self.leg_l.smallleg2.longth
        c = self.leg_l.smallleg2.longth**2-self.leg_l.smallleg1.longth**2-p**2-q**2-2*q*self.leg_l.smallleg1.longth


        z1 = 0.5 * (-b + sqrt(b**2-4*a*c))/a
        z2 = 0.5 * (-b - sqrt(b**2-4*a*c))/a

        if 2 * atan(z1) > pi/2 :
                self.leg_l.smallleg2.angle = 2 * atan(z1)
        else:
                self.leg_l.smallleg2.angle = 2 * atan(z2)
        self.leg_l.calculate()
        


    def backward(self):
        self.leg_r.bigleg1.position[0]=self.gimbal.longth*cos(self.gimbal.angle)
        self.leg_r.bigleg1.position[2]=self.gimbal.longth*sin(self.gimbal.angle)

        self.leg_r.bigleg2.position[0]=-self.gimbal.longth*cos(self.gimbal.angle)
        self.leg_r.bigleg2.position[2]=-self.gimbal.longth*sin(self.gimbal.angle)

        p = self.leg_r.wheel.position_target[2]-self.leg_r.bigleg1.position[2]
        q = self.leg_r.wheel.position_target[0]-self.leg_r.bigleg1.position[0]

        a = q**2+p**2+self.leg_r.bigleg1.longth**2-self.leg_r.smallleg1.longth**2+2*self.leg_r.bigleg1.longth*q
        b = -4*self.leg_r.bigleg1.longth*p
        c = q**2+p**2+self.leg_r.bigleg1.longth**2-self.leg_r.smallleg1.longth**2-2*self.leg_r.bigleg1.longth*q
        z1 = 0.5 * (-b + sqrt(b**2-4*a*c))/a
        z2 = 0.5 * (-b - sqrt(b**2-4*a*c))/a

        if 2 * atan(z1) < -pi/2 :
                self.leg_l.bigleg1.angle_target = 2 * atan(z1)
        else:
                self.leg_l.bigleg1.angle_target = 2 * atan(z2)
        
        p = self.leg_r.wheel.position_target[2]-self.leg_r.bigleg2.position[2]
        q = self.leg_r.wheel.position_target[0]-self.leg_r.bigleg2.position[0]

        a = q**2+p**2+self.leg_r.bigleg2.longth**2-self.leg_r.smallleg1.longth**2+2*self.leg_r.bigleg2.longth*self.leg_r.wheel.position_target[0]
        b = -4*self.leg_r.bigleg2.longth*self.leg_r.wheel.position_target[2]
        c = q**2+p**2+self.leg_r.bigleg2.longth**2-self.leg_r.smallleg1.longth**2-2*self.leg_r.bigleg2.longth*self.leg_r.wheel.position_target[0]
        z1 = 0.5 * (-b + sqrt(b**2-4*a*c))/a
        z2 = 0.5 * (-b - sqrt(b**2-4*a*c))/a

        if 2 * atan(z1) > -5*pi/6 and 2 * atan(z1) < -pi/2:
                self.leg_r.bigleg2.angle_target = 2 * atan(z1)
        else:
                self.leg_r.bigleg2.angle_target = 2 * atan(z2)
######################################################################################################################################################
        self.leg_l.bigleg1.position[0]=self.gimbal.longth*cos(self.gimbal.angle)
        self.leg_l.bigleg1.position[2]=self.gimbal.longth*sin(self.gimbal.angle)

        self.leg_l.bigleg2.position[0]=-self.gimbal.longth*cos(self.gimbal.angle)
        self.leg_l.bigleg2.position[2]=-self.gimbal.longth*sin(self.gimbal.angle)

        p = self.leg_l.wheel.position_target[2]-self.leg_l.bigleg1.position[2]
        q = self.leg_l.wheel.position_target[0]-self.leg_l.bigleg1.position[0]

        a = q**2+p**2+self.leg_l.bigleg1.longth**2-self.leg_l.smallleg1.longth**2+2*self.leg_l.bigleg1.longth*self.leg_l.wheel.position_target[0]
        b = 4*self.leg_l.bigleg1.longth*self.leg_l.wheel.position_target[2]
        c = q**2+p**2+self.leg_l.bigleg1.longth**2-self.leg_l.smallleg1.longth**2-2*self.leg_l.bigleg1.longth*self.leg_l.wheel.position_target[0]
        z1 = 0.5 * (-b + sqrt(b**2-4*a*c))/a
        z2 = 0.5 * (-b - sqrt(b**2-4*a*c))/a

        if 2 * atan(z1) > -pi/3 and 2 * atan(z1) < pi/2:
                self.leg_l.bigleg1.angle_target = 2 * atan(z1)
        else:
                self.leg_l.bigleg1.angle_target = 2 * atan(z2)
        
        p = self.leg_l.wheel.position_target[2]-self.leg_l.bigleg2.position[2]
        q = self.leg_l.wheel.position_target[0]-self.leg_l.bigleg2.position[0]

        a = q**2+p**2+self.leg_l.bigleg2.longth**2-self.leg_l.smallleg1.longth**2+2*self.leg_l.bigleg2.longth*self.leg_l.wheel.position_target[0]
        b = 4*self.leg_l.bigleg2.longth*self.leg_l.wheel.position_target[2]
        c = q**2+p**2+self.leg_l.bigleg2.longth**2-self.leg_l.smallleg1.longth**2-2*self.leg_l.bigleg2.longth*self.leg_l.wheel.position_target[0]
        z1 = 0.5 * (-b + sqrt(b**2-4*a*c))/a
        z2 = 0.5 * (-b - sqrt(b**2-4*a*c))/a

        if 2 * atan(z1) > -5*pi/6 and 2 * atan(z1) < -pi/2:
                self.leg_l.bigleg2.angle_target = 2 * atan(z1)
        else:
                self.leg_l.bigleg2.angle_target = 2 * atan(z2)
import mujoco as mj
import math
from math import cos, sin, atan, atan2, sqrt, pi

Model = mj.MjModel.from_xml_path("legwheel.xml")
Data = mj.MjData(Model)




class gimbal:
    def __init__(self, body=None, position=[0,0,0], position_target=[0,0,0], angle=0, angle_target=0, vocity=0, force=0, gyro=[None,None,None], accel=[None,None,None], longth=0,quat=[None,None,None,None], sensor_gyro_id=None, sensor_quat_id=None,sensor_pos_id=None,sensor_vel_id=None):
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
        self.quat = quat

        self.sensor_gyro_id = sensor_gyro_id
        self.sensor_quat_id = sensor_quat_id
        self.sensor_pos_id = sensor_pos_id
        self.sensor_vel_id = sensor_vel_id

    def read_gyro(self):
        start_index = Model.sensor_adr[self.sensor_gyro_id]
        data_dim = Model.sensor_dim[self.sensor_gyro_id]
        self.gyro = Data.sensordata[start_index : start_index + data_dim]
    def read_quat(self):
        start_index = Model.sensor_adr[self.sensor_quat_id]
        data_dim = Model.sensor_dim[self.sensor_quat_id]
        self.quat = Data.sensordata[start_index : start_index + data_dim]
    
          
    def read_pos(self):
        start_index = Model.sensor_adr[self.sensor_pos_id]
        data_dim = Model.sensor_dim[self.sensor_pos_id]
        self.position = Data.sensordata[start_index : start_index + data_dim]
    def read_vel(self):
        start_index = Model.sensor_adr[self.sensor_vel_id]
        data_dim = Model.sensor_dim[self.sensor_vel_id]
        self.vocity = Data.sensordata[start_index : start_index + data_dim]

class virtual_leg_l:
    def __init__(self, body_name=None, angle=0, longth=0, vocity=None, v_longth=0):
        self.body_name = body_name
        self.angle = angle
        self.longth = longth
        self.vocity = vocity
        self.v_longth = v_longth
class virtual_leg_r:
    def __init__(self, body_name=None, angle=0, longth=0, vocity=None, v_longth=0):
        self.body_name = body_name
        self.angle = angle
        self.longth = longth
        self.vocity = vocity
        self.v_longth = v_longth
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
        self.angle = Data.sensordata[start_index : start_index + data_dim]/1
    def read_vel(self):
        start_index = Model.sensor_adr[self.sensor_vel_id]
        data_dim = Model.sensor_dim[self.sensor_vel_id]
        self.vocity = Data.sensordata[start_index : start_index + data_dim]/1
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
        self.angle = Data.sensordata[start_index : start_index + data_dim]/1
    def read_vel(self):
        start_index = Model.sensor_adr[self.sensor_vel_id]
        data_dim = Model.sensor_dim[self.sensor_vel_id]
        self.vocity = Data.sensordata[start_index : start_index + data_dim]/1
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
        self.angle = Data.sensordata[start_index : start_index + data_dim]/1
    def read_vel(self):
        start_index = Model.sensor_adr[self.sensor_vel_id]
        data_dim = Model.sensor_dim[self.sensor_vel_id]
        self.vocity = Data.sensordata[start_index : start_index + data_dim]/1
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
        self.angle = Data.sensordata[start_index : start_index + data_dim]/1
    def read_vel(self):
        start_index = Model.sensor_adr[self.sensor_vel_id]
        data_dim = Model.sensor_dim[self.sensor_vel_id]
        self.vocity = Data.sensordata[start_index : start_index + data_dim]/1
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
        self.angle = Data.sensordata[start_index : start_index + data_dim]/1
    def read_vel(self):
        start_index = Model.sensor_adr[self.sensor_vel_id]
        data_dim = Model.sensor_dim[self.sensor_vel_id]
        self.vocity = Data.sensordata[start_index : start_index + data_dim]/1
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
        self.angle = Data.sensordata[start_index : start_index + data_dim]/1
    def read_vel(self):
        start_index = Model.sensor_adr[self.sensor_vel_id]
        data_dim = Model.sensor_dim[self.sensor_vel_id]
        self.vocity = Data.sensordata[start_index : start_index + data_dim]/1
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
        self.virtual_leg.angle = atan2(0-self.wheel.position[0],self.wheel.position[2]-0)
        self.virtual_leg.longth = sqrt(self.wheel.position[2]**2 +self.wheel.position[0]**2)
        self.virtual_leg.vocity = (self.bigleg1.vocity)*self.bigleg1.longth/self.virtual_leg.longth*cos(self.smallleg1.angle+self.virtual_leg.angle)*sin(self.smallleg1.angle-self.bigleg1.angle)
        +self.bigleg2.vocity*self.bigleg2.longth/self.virtual_leg.longth*cos(self.smallleg2.angle+self.virtual_leg.angle)*sin(self.smallleg2.angle-self.bigleg2.angle)
        self.virtual_leg.v_longth =(self.bigleg1.vocity)*self.bigleg1.longth/self.virtual_leg.longth*sin(self.smallleg1.angle+self.virtual_leg.angle)*sin(self.smallleg1.angle-self.bigleg1.angle)
        +self.bigleg2.vocity*self.bigleg2.longth/self.virtual_leg.longth*sin(self.smallleg2.angle+self.virtual_leg.angle)*sin(self.smallleg2.angle-self.bigleg2.angle)
        
class leg_r:
    def __init__(self):
        self.bigleg1 = bigleg1_r()
        self.bigleg2 = bigleg2_r()
        self.smallleg1 = smallleg1_r()
        self.smallleg2 = smallleg2_r()
        self.wheel = wheel_r()
        self.virtual_leg = virtual_leg_r()
    def calculate(self):
        self.virtual_leg.angle = atan2(0-self.wheel.position[0],self.wheel.position[2]-0)
        self.virtual_leg.longth = sqrt(self.wheel.position[2]**2 +self.wheel.position[0]**2)
        self.virtual_leg.vocity = self.bigleg1.vocity*self.bigleg1.longth/self.virtual_leg.longth*cos(self.smallleg1.angle+self.virtual_leg.angle)*sin(self.smallleg1.angle-(self.bigleg1.angle))
        +(self.bigleg2.vocity)*self.bigleg2.longth/self.virtual_leg.longth*cos(self.smallleg2.angle+self.virtual_leg.angle)*sin(self.smallleg2.angle-(self.bigleg2.angle))
        self.virtual_leg.v_longth = self.bigleg1.vocity*self.bigleg1.longth/self.virtual_leg.longth*sin(self.smallleg1.angle+self.virtual_leg.angle)*sin(self.smallleg1.angle-(self.bigleg1.angle))
        +(self.bigleg2.vocity)*self.bigleg2.longth/self.virtual_leg.longth*sin(self.smallleg2.angle+self.virtual_leg.angle)*sin(self.smallleg2.angle-(self.bigleg2.angle))


class robot:
    def __init__(self):
        self.gimbal = gimbal()
        self.leg_l = leg_l()
        self.leg_r = leg_r()

    def read(self):
        self.gimbal.read_gyro()
        self.gimbal.read_quat()
        self.gimbal.read_pos()
        self.gimbal.read_vel()
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
        self.leg_l.bigleg1.read_force()
        self.leg_l.bigleg2.read_force()
        self.leg_r.bigleg1.read_force()
        self.leg_r.bigleg2.read_force()
        self.leg_l.wheel.read_force()
        self.leg_r.wheel.read_force()
    def write(self,gyro_id, quat_id,pos_id,vel_id,
                pos1_id, pos2_id, pos3_id, pos4_id,pos5_id, pos6_id, 
                vel1_id, vel2_id, vel3_id, vel4_id, vel5_id, vel6_id,
                force1_id, force2_id, force3_id, force4_id, force5_id, force6_id,
                bigleg, smallleg, r,d):
        
        self.gimbal.sensor_gyro_id = gyro_id
        self.gimbal.sensor_quat_id = quat_id
        self.gimbal.sensor_pos_id = pos_id
        self.gimbal.sensor_vel_id = vel_id

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

        self.leg_r.smallleg1.position[0] = self.leg_r.bigleg1.position[0] + self.leg_r.bigleg1.longth * cos(self.leg_r.bigleg1.angle)
        self.leg_r.smallleg1.position[2] = self.leg_r.bigleg1.position[2] + self.leg_r.bigleg1.longth * sin(self.leg_r.bigleg1.angle)

        self.leg_r.smallleg2.position[0] = self.leg_r.bigleg2.position[0] + self.leg_r.bigleg2.longth * cos(self.leg_r.bigleg2.angle)
        self.leg_r.smallleg2.position[2] = self.leg_r.bigleg2.position[2] + self.leg_r.bigleg2.longth * sin(self.leg_r.bigleg2.angle)

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

        if 2 * atan(z1) < pi/2 :
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

        if 2 * atan(z1) < pi/2 :
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

        a = q**2+p**2+self.leg_r.bigleg1.longth**2-self.leg_r.smallleg1.longth**2+2*self.leg_r.bigleg1.longth*self.leg_r.wheel.position_target[0]
        b = -4*self.leg_r.bigleg1.longth*p
        c = q**2+p**2+self.leg_r.bigleg1.longth**2-self.leg_r.smallleg1.longth**2-2*self.leg_r.bigleg1.longth*self.leg_r.wheel.position_target[0]
        z1 = 0.5 * (-b + sqrt(b**2-4*a*c))/a
        z2 = 0.5 * (-b - sqrt(b**2-4*a*c))/a

        if 2 * atan(z1) < pi/2 :
                self.leg_r.bigleg1.angle_target = 2 * atan(z1)
        else:
                self.leg_r.bigleg1.angle_target = 2 * atan(z2)
        
        p = self.leg_r.wheel.position_target[2]-self.leg_r.bigleg2.position[2]
        q = self.leg_r.wheel.position_target[0]-self.leg_r.bigleg2.position[0]

        a = q**2+p**2+self.leg_r.bigleg2.longth**2-self.leg_r.smallleg1.longth**2+2*self.leg_r.bigleg2.longth*self.leg_r.wheel.position_target[0]
        b = -4*self.leg_r.bigleg2.longth*self.leg_r.wheel.position_target[2]
        c = q**2+p**2+self.leg_r.bigleg2.longth**2-self.leg_r.smallleg1.longth**2-2*self.leg_r.bigleg2.longth*self.leg_r.wheel.position_target[0]
        z1 = 0.5 * (-b + sqrt(b**2-4*a*c))/a
        z2 = 0.5 * (-b - sqrt(b**2-4*a*c))/a

        if 2 * atan(z1) > pi/2:
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
        b = -4*self.leg_l.bigleg1.longth*self.leg_l.wheel.position_target[2]
        c = q**2+p**2+self.leg_l.bigleg1.longth**2-self.leg_l.smallleg1.longth**2-2*self.leg_l.bigleg1.longth*self.leg_l.wheel.position_target[0]
        z1 = 0.5 * (-b + sqrt(b**2-4*a*c))/a
        z2 = 0.5 * (-b - sqrt(b**2-4*a*c))/a

        if 2 * atan(z1) < pi/2:
                
                self.leg_l.bigleg1.angle_target = 2 * atan(z1)
        else:
                self.leg_l.bigleg1.angle_target = 2 * atan(z2)
        
        p = self.leg_l.wheel.position_target[2]-self.leg_l.bigleg2.position[2]
        q = self.leg_l.wheel.position_target[0]-self.leg_l.bigleg2.position[0]

        a = q**2+p**2+self.leg_l.bigleg2.longth**2-self.leg_l.smallleg1.longth**2+2*self.leg_l.bigleg2.longth*self.leg_l.wheel.position_target[0]
        b = -4*self.leg_l.bigleg2.longth*self.leg_l.wheel.position_target[2]
        c = q**2+p**2+self.leg_l.bigleg2.longth**2-self.leg_l.smallleg1.longth**2-2*self.leg_l.bigleg2.longth*self.leg_l.wheel.position_target[0]
        z1 = 0.5 * (-b + sqrt(b**2-4*a*c))/a
        z2 = 0.5 * (-b - sqrt(b**2-4*a*c))/a

        if 2 * atan(z1) > pi/2:
                self.leg_l.bigleg2.angle_target = 2 * atan(z1)
        else:
                self.leg_l.bigleg2.angle_target = 2 * atan(z2)



class LowPassFilter:
    def __init__(self, alpha=0.1):
        """
        初始化低通滤波器
        :param alpha: 滤波系数 (0.0 < alpha <= 1.0)
                      值越小，平滑度越高，延迟越大。
                      值越大，灵敏度越高，噪声越多。
        """
        self.alpha = alpha
        self.last_output = None  # 存储上一次的滤波结果

    def update(self, current_input):
        """
        计算新的滤波值
        :param current_input: 当前传感器读到的原始值
        :return: 滤波后的值
        """
        # 第一次运行时，直接使用当前值作为初始值
        if self.last_output is None:
            self.last_output = current_input
            return current_input

        # 核心算法：一阶低通滤波
        # 新值 = α * 当前输入 + (1 - α) * 上次输出
        filtered_value = (self.alpha * current_input) + \
                         ((1 - self.alpha) * self.last_output)
        
        # 更新状态
        self.last_output = filtered_value
        
        return filtered_value

    def reset(self):
        """重置滤波器"""
        self.last_output = None
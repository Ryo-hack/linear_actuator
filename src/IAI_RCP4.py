#!/usr/bin/env python
# -*- coding: utf-8 -*-
from re import L, S
import re
import rospy
import sys
import rosparam
import time
import serial
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

class LinearAct :
    def __init__(self):
        self.point  = 0.0
        self.velocity  =0.0
        self.current  =0.0
        self.alm_msg = 0.0
        self.current_pub = rospy.Publisher("IHI_linearActuator_Current", Float32, queue_size = 10)
        self.point_pub = rospy.Publisher("IHI_linearActuator_Point",  Float32, queue_size=10)
        self.velocity_pub = rospy.Publisher("IHI_linearActuator_Velocity", Float32, queue_size = 10)
        self.port_name = rospy.get_param('~port','/dev/ttyUSB1')
        if len(sys.argv) == 2 :
            self.port_name  = sys.argv[1]
        self.ser = serial.Serial(self.port_name,230400,timeout=0.1)
        rospy.loginfo("Connected on %s" % (self.port_name) )
        
    def Actuator_Init(self):
        rospy.loginfo("hardware Initializing....")
        self.ser.write(":01050427FF00D0\r\n")#PIO 有効　
        self.ser.write(":010504270000EE\r\n") #PIO 無効
        msg = self.ser.readline()
        if msg ==msg:
            rospy.loginfo(msg)
            rospy.loginfo("hardware Initialize OK...")
            self.ser.write(":01050414FF00E3\r\n")#教示 
            self.ser.write(":0105041400001F\r\n") #通常
            msg = self.ser.readline()
            rospy.loginfo(msg)
            self.ser.write(":01050403FF00F4\r\n")
            msg = self.ser.readline()
            rospy.loginfo(msg)
            self.ser.write(":01050407FF00F0\r\n")
            msg = self.ser.readline()
            rospy.loginfo(msg)
            self.ser.write(":010504070000EF\r\n")
            msg = self.ser.readline()
            rospy.loginfo(msg)
            rospy.loginfo("hardware Initialized....")
            time.sleep(1)
        else:
            rospy.loginfo("hardware Initialize false ")
            sys.exit()
        return 

    def home(self):
            self.ser.write(":01060D001000DC\r\n")
            msg = self.ser.readline()
            rospy.loginfo(msg)
            self.ser.write(":01060D001010CC\r\n")
            msg = self.ser.readline()
            rospy.loginfo(msg)
            
            while 1:
                self.Pnow()
                if self.point == 0.0 :
                        break
                else :    
                    print("runnning")

    def abs_move(self,target_point,target_velocity,target_acceleration):
        target_point = str(hex(100*target_point ))[2:].zfill(4)
        target_velocity = str(hex(100*target_velocity))[2:].zfill(4)
        target_acceleration =str(hex(int(100*target_acceleration)))[2:].zfill(4)
        target_msg =(str(":011099000009120000")+ target_point +str("0000") +str("000A") +str("0000")  + target_velocity + target_acceleration +str("00000000"))

        list = re.split('(..)',target_msg[1:])[1::2]
        LRC = 0
        for i in range(len(list)):
            LRC =LRC +int(list[i],16)
        LRC = str(hex(int(hex(LRC),16)-int( ("0x")+str(10**(len(str(LRC)[:2])+1)),16)))[-2:]
        target_msg = (target_msg + LRC +str("\r\n")).upper()

        rospy.loginfo(target_msg)
        self.ser.write(target_msg)

    def relate_move(self,target_point,target_velocity,target_acceleration):
        Point = target_point
        target_point = str(hex(100*target_point ))[2:].zfill(4)
        target_velocity = str(hex(100*target_velocity))[2:].zfill(4)
        target_acceleration =str(hex(int(100*target_acceleration)))[2:].zfill(4)
        target_msg =(str(":011099000009120000")+ target_point +str("0000") +str("000A") +str("0000")  + target_velocity + target_acceleration +str("00000008"))
        list = re.split('(..)',target_msg[1:])[1::2]
        LRC = 0
        for i in range(len(list)):
            LRC =LRC +int(list[i],16)
        LRC = str(hex(int(hex(LRC),16)-int( ("0x")+str(10**(len(str(LRC)[:2])+1)),16)))[-2:]
        target_msg = (target_msg + LRC +str("\r\n")).upper()
        rospy.loginfo(target_msg)
        self.ser.write(target_msg)
        while 1:
            self.Pnow()
            if self.point ==87.99:#float(Point):
                break
            else :    
                print("runnning")

    def stop (self,time):
        t_start = rospy.get_time()
        while 1:
            self.Pnow()
            t_now = rospy.get_time()
            Time = t_now - t_start
            if Time >= time:
                break
            else :    
                print("wait")


    def ERC_check (self):
        msg ="010504270000"#"010504270000" 
        list = re.split('(..)',msg[1:])[1::2]
        LRC = 0
        for i in range(len(list)):
            LRC =LRC +int(list[i],16)
        LRC = str(hex(int(hex(LRC),16)-int( ("0x")+str(10**(len(str(LRC)[:2])+1)),16)))[-2:]
        msg = (msg + LRC +str("\r\n")).upper()
        rospy.loginfo(msg)

    def Pnow(self):
        rate = rospy.Rate(5)
        self.ser.write(":0103900000026A\r\n")
        _point = self.ser.readline()
        self.point =0.01*(float(str(int(_point[7:11],16)) + str(int(_point[11:15],16))))
        rospy.loginfo(self.point)
        self.point_pub.publish(self.point)   
        rate.sleep()    

    def Vnow(self):
        self.ser.write(":0103900A000260\r\n")
        _velocity = self.ser.readline()
        self.velocity =0.01*(float(str(int(_velocity[7:11],16)) + str(int(_velocity[11:15],16))))
        rospy.loginfo(self.velocity)
        self.velocity_pub.publish(self.velocity)


    def Cnow(self):
        self.ser.write(":0103900C00025E\r\n")
        _current = self.ser.readline()
        self.current =0.01*(float(str(int(_current[7:11],16)) + str(int(_current[11:15],16))))
        rospy.loginfo(self.current)  
        self.current_pub.publish(self.current)

    def Alm_now(self):
        self.ser.write(":01039002000169\r\n")
        self.alm_msg = self.ser.readline()
        #if :\
        #    flag = True
       # else:
        #       flag = False
       # return flag 

    def Alm_reset():
        rospy.loginfo("try_alam_reset")
        self.ser.write(":01050407FF00F0\r\n")
        self.ser.write(":010504070000EF\r\n")
        self.ser.write(":01039002000169\r\n")
        msg = self.ser.readline()
        if msg == msg :
            rospy.loginfo("alam_reset_ok")
        else :
            rospy.loginfo("alam_reset_false")
            sys.exit()


if __name__ == '__main__':
    rospy.init_node("IHI_linear_actuator")

    LinearAct= LinearAct()
    LinearAct.Actuator_Init()
    #LinearAct.ERC_check()
    #LinearAct.Alm_reset()
    #LinearAct.abs_move(50,100,0.3)
    rate = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        LinearAct.home()
        LinearAct.relate_move(88,50,0.3)
        LinearAct.stop(3)

        #LinearAct.relate_move()
        #LinearAct.Pnow()
        #LinearAct.Vnow()
        #LinearAct.Cnow()

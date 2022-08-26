#!/usr/bin/env python
# -*- coding: utf-8 -*-
from re import L, S
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
        self.current_pub = rospy.Publisher("IHI_linearActuator_Current", Float32MultiArray, queue_size = 10)
        self.point_pub = rospy.Publisher("IHI_linearActuator_Point",  Float32MultiArray, queue_size=10)
        self.velocity_pub = rospy.Publisher("IHI_linearActuator_Velocity", Float32MultiArray, queue_size = 10)
        self.port_name = rospy.get_param('~port','/dev/ttyUSB0')
        if len(sys.argv) == 2 :
            self.port_name  = sys.argv[1]
        self.ser = serial.Serial(self.port_name,9600,timeout=0.1)
        rospy.loginfo("Connected on %s" % (self.port_name) )
        
    def Actuator_Init(self):
        rospy.loginfo("hardware Initializing....")
        self.ser.write(":01050427FF00D0\r\n")
        msg = self.ser.readline()
        print (type(msg))
        if msg ==msg:
            rospy.loginfo(msg)
            rospy.loginfo("hardware Initialize OK...")
            self.ser.write(":01050414FF00E3\r\n")
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
    
    def abs_move(self):
        self.ser.write(":010504070000EF\r\n")

    def relate_move(self):
        self.ser.write(":01109900000912000003E80000000A00002710001E00000008E9\r\n")


    def Pnow(self):
        self.ser.write(":0103900000026A\r\n")
        self.point = self.ser.readline()
        rospy.loginfo(self.point)

    def Vnow(self):
        self.ser.write(":0103900A000260\r\n")
        self.velocity = self.ser.readline()
        rospy.loginfo(self.velocity)
    
    def Cnow(self):
        self.ser.write(":0103900C00025E\r\n")
        self.current = self.ser.readline()
        rospy.loginfo(self.current)
    
    def Alm_now(self):
        self.ser.write(":01039002000169\r\n")
        self.alm_msg = self.ser.readline()
        #if :
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

    def publisher(self):
        rospy.loginfo(self.force_data)
        rospy.loginfo(vector)
   


if __name__ == '__main__':
    rospy.init_node("IHI_linear_actuator")
    LinearAct= LinearAct()
    LinearAct.Actuator_Init()
    LinearAct.relate_move()
    rate = rospy.Rate(100000)
    while not rospy.is_shutdown():
        LinearAct.Pnow()
        rate.sleep()    
#!/usr/bin/env python
#/////////////////////////////////////////////////////////////
#
# ROS related imports
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from rospy_tutorials.msg import Floats

# Other Imports
import sys
import os.path
import math
import time
import datetime
import serial
import crcmod.predefined

import RPi.GPIO as gpio

poll_interval = 10
thrusters = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

#
# ACTUATOR related functions
#/////////////////////////////////////////////////////////////////
# Deals with subscription callbacks and other actuator related functions
def thruster_callback(msg):
    global thrusters
    if len(msg.data) is 10:
         for i in range(10):
             thrusters[i] = msg.data[i]


def build_actuator_msg(motors):
  data = 'M'
  for i in range(10):
    data += chr(int(127*(motors[i]+1)))
  data += 'S'
  for i in range(5):
    data += chr(0)
  data += 'R'

  crc = crc16(data)
  data += chr(crc >> 8)
  data += chr(crc & 0xff)

  msg = '12'
  for i in list(data):
    if(i == chr(18)) or (i == chr(255)) or (i == chr(19)):
      msg += 'ff'
    msg += i.encode('hex')
  msg += '13'
  return msg

#
# GPIO related functions
#/////////////////////////////////////////////////////////////////
# Mission Switch on GPIO4, there is pull up attached so by default input is high 
# when pressed input is low
#def mission_engage():
#    print("Engage!")
  
#
#/////////////////////////////////////////////////////////////////
# Startup 
#/////////////////////////////////////////////////////////////////
# GPIO Startup
# Input
gpio.setmode(gpio.BCM)
gpio.setup(4, gpio.IN)

#
# ROS Startup
# //////////////////////////////////////////////////////////////////////
pubMission = rospy.Publisher('mission', Bool, queue_size=10)
subThrusters = rospy.Subscriber('thrusters', Floats, thruster_callback)

rate = rospy.Rate(1000 / poll_interval)

#
# Setup actuator board
#
crc16 = crcmod.predefined.mkCrcFun('crc-16')
ser = serial.Serial(port='/dev/ttyS0', baudrate=38400, timeout=1)

#
# ////////////////////////////////////////////////////////////////////////
# Main Loop
# ////////////////////////////////////////////////////////////////////////
#
previousDisplayTimeS = time.time()

while True:
  currentTimeS = time.time()
  #print("Time %+6.3f" % (currentTimeS))
  # 
  
  mission = False if gpio.input(4) else True

  pubMission.publish(mission)

  # 
  # At reduced rate write to actuator board
  #
  if not mission:
    thrusters = [0 for i in range(10)]

  msg = build_actuator_msg(thrusters)
  ser.write(msg.decode('hex'))

  if ((currentTimeS - previousDisplayTimeS) > 0.1):
    #print("IMU get data")
    #print("Acc x: %+6.2f y: %+6.2f z: %+6.2f length: %+6.3f" % (acc[0], acc[1], acc[2], math.sqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]) ) ) 
    previousDisplayTimeS = currentTimeS
  
  # release task
  # use rate once ROS is working and booted up
  rate.sleep()
  #timeRemaining = poll_interval/1000.0 - (time.time() - currentTimeS)
  #if (timeRemaining > 0):
  #  time.sleep(timeRemaining)

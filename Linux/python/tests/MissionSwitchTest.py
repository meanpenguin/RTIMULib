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

import RPi.GPIO as gpio

poll_interval = 10

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
# ////////////////////////////////////////////////////////////////////////
# Main Loop
# ////////////////////////////////////////////////////////////////////////
#
previousDisplayTimeS = time.time()

while True:
  currentTimeS = time.time()
  print("Time %+6.3f" % (currentTimeS))
    
  mission = False if gpio.input(4) else True
  if mission:
    print("Mission ON!")
  else:
    print("Mission OFF!")

  # release task
  timeRemaining = poll_interval/1000.0 - (time.time() - currentTimeS)
  if (timeRemaining > 0):
    time.sleep(timeRemaining)

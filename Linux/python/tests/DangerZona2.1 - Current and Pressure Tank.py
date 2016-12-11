#!/usr/bin/env python
#/////////////////////////////////////////////////////////////
#
# ROS related imports
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from rospy_tutorials.msg import Floats


# GPIO related imports
from gpiozero import MCP3208
#from gpiozero import Button
#from gpiozero import DigitalOutputDevice

# Other Imports
import sys
import os.path
import math
import time
import datetime

def limVal(var, lower, upper):
  if var < lower:
    var = lower
  elif var > upper:
    var = upper
  return var

def low_pass(curr, prev, alpha):
  return curr * alpha + prev * (1 - alpha)


#
#/////////////////////////////////////////////////////////////////
# Startup 
#/////////////////////////////////////////////////////////////////
#
# MCP3208 A/D Startup
#////////////////////////////////////////////////////////////////
# MCP3008(channel=0, clock_pin=11, mosi_pin=10, miso_pin=9, select_pin=8)
# 8 channel A/D microchip 
# usually Chip Select is not connected to pin 22 but to pin 8 or 7
# if new sensor board is made, change that pin to new chip select connection
# do not change mosi_pin
# do not change clock pin
# select_pin is same for a single A/D chip, so keep same for all channels
# One input is converted at a time
ch0=MCP3208(channel=0, clock_pin=11, mosi_pin=10, miso_pin=9, select_pin=22)
ch4=MCP3208(channel=4, clock_pin=11, mosi_pin=10, miso_pin=9, select_pin=22)
# interval for reading MCP A/D converter, needs to be same or slower than IMU rate
# in seconds
AmpHour = 0.0
pressureTorpedo = 0.0

#
poll_interval = 100
print("Recommended Poll Interval: %dmS\n" % poll_interval)

#
# ROS Startup
# //////////////////////////////////////////////////////////////////////
pubAmpHour = rospy.Publisher('AmpHour', Float64, queue_size=10)
pubPressureTorpedo = rospy.Publisher('pressureTorpedo`', Float64, queue_size=10)
rate = rospy.Rate(1000 / poll_interval)

#
# ////////////////////////////////////////////////////////////////////////
# Main Loop
# ////////////////////////////////////////////////////////////////////////
#
# 

while True:
  currentTimeS = time.time()
  #print("Time %+6.3f" % (currentTimeS))

  #
  # Pressure Sensor
   #      
  # NBP series unamplified 0..150psi
  # Output 0..130mV
  reading0 = ch0.raw_value
  voltage0 = reading0 * 3.3 / 4096
  # conversion mv/V/full scale span 0...150psi 21.7 26.0 30.0
  #            remove zero     convert to mv       nominal sensitivity full range   
  # At ambient pressure the reading is close to 4096/2; Offset is 2069 +/-1
  # Calibration not verified yet !!!! keep 2069 though
  pressureTorpedo = (reading0 - 2069) * 3.3 / 4096 * 1000 / 26 * 150
  #
  # Battery Current Sensor
  #        
  reading4 = ch4.raw_value
  voltage4 = reading4 * 3.3 / 4096
  # Some calibration here
  # current =  (voltage4-0.001) * 1000
  current = 0.0
  AmpHour = AmpHour + ((currentTimeS - previousTimeMCPS) * current / 3600)      
  
  #
  # Need to send it to ROS master somehow
  #
  if not rospy.is_shutdown():
    pubAmpHour.publish(AmpHour)
    pubPressureTorpedo.publish(pressureTorpedo)
     
  #        
  # At reduced rate display data (10Hz)
  # 
  if ((currentTimeS - previousDisplayTimeS) > 0.1):
    print("Torpedo Pressure Reading = %2.2f psi" % (pressureTorpedo) )
    print("AmpHour consumed %3.3f [Ahr]" % (AmpHour) )
    previousDisplayTimeS = currentTimeS

  # release task
  # use rate once ROS is working and booted up
  rate.sleep()
  #timeRemaining = poll_interval/1000.0 - (time.time() - currentTimeS)
  #if (timeRemaining > 0):
  #  time.sleep(timeRemaining)

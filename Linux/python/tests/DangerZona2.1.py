#!/usr/bin/env python
#/////////////////////////////////////////////////////////////
#
# ROS related imports
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from rospy_tutorials.msg import Floats


# RTIMU reltate imports
import RTIMU

# GPIO related imports
from gpiozero import MCP3208
#from gpiozero import Button
#from gpiozero import DigitalOutputDevice
import RPi.GPIO as gpio

# Other Imports
import sys
import os.path
import math
import time
import datetime
import serial
import crcmod.predefined

sys.path.append('.')

displayIMU = True
staticpressure = 1012.0


def limVal(var, lower, upper):
  if var < lower:
    var = lower
  elif var > upper:
    var = upper
  return var

def low_pass(curr, prev, alpha):
  return curr * alpha + prev * (1 - alpha)


# Setup actuator board
crc16 = crcmod.predefined.mkCrcFun('crc-16')
ser = serial.Serial(port='/dev/ttyS0', baudrate=38400, timeout=1)

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

thrusters = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

#
# Pressure Sensor related
#//////////////////////////////////////////////////////////
#
#  computeHeight() - the conversion uses the formula:
#  h = (T0 / L0) * ((p / P0)**(-(R* * L0) / (g0 * M)) - 1)
#  where:
#  h  = height above sea level
#  T0 = standard temperature at sea level = 288.15
#  L0 = standard temperatur elapse rate = -0.0065
#  p  = measured pressure
#  P0 = static pressure = 1013.25
#  g0 = gravitational acceleration = 9.80665
#  M  = mloecular mass of earth's air = 0.0289644
#  R* = universal gas constant = 8.31432
#  Given the constants, this works out to:
#  h = 44330.8 * (1 - (p / P0)**0.190263)
#
def computeHeight(pressure, staticpressure):
    if (staticpressure > 0):
      return 44330.8 * (1 - pow(pressure / staticpressure, 0.190263));
    else:
      return 0.0;
  
#   
# Compute Depth
# Saltwater density is 1.025 kg/l
# Freshwater density is 1.0 kg/l
# Gravity is 9.80665 m/s/s
# delta P = rho g h
# see level pressure in average: 1013.25
# 1 millibar = 100 pascal [kg/m/s/s]
# 100/9.8/1.025
#
# http://www.seabird.com/document/an69-conversion-pressure-depth
#
#The gravity variation with latitude and pressure is computed as:
# g (m/sec2) = 9.780318 * (1.0 + ( 5.2788E-3  + 2.36E-5  * x) * x ) + 1.092E-6  * p
#where
# x = (sin (latitude / 57.29578) ) ^ 2
# p = pressure (decibars)
# San Diego: 32.72  p=1013.25/1000/10 x=0.2922 g=9.7954
#Then, depth is calculated from pressure:
# depth (meters) = ((((-1.82e-15  * p + 2.279e-10 ) * p - 2.2512e-5 ) * p + 9.72659) * p) / g
#where
#p = pressure (decibars)
#g = gravity (m/sec2)
#
def computeDepthFreshWater(pressure, staticpressure):
    return (pressure - staticpressure) * 0.01019716;    

def computeDepthSaltWater(pressure, staticpressure):
    p=(pressure-staticpressure)/100
    return ((( (-1.82e-15*p + 2.279e-10 )*p - 2.2512e-5 )*p + 9.72659) * p) / 9.7954;    

#
# GPIO related functions
#/////////////////////////////////////////////////////////////////
# Mission Switch on GPIO4, there is pull up attached so by default input is high 
# when pressed input is low
def mission_engage():
    print("Engage!")



#
# ACTUATOR related functions
#/////////////////////////////////////////////////////////////////
# Deals with subscription callbacks and other actuator related functions
def thruster_callback(msg):
    global thrusters
    if len(msg.data) is 10:
         for i in range(10):
             thrusters[i] = msg.data[i]

deltaActuator = 0.02

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
deltaMCP = 0.1
AmpHour = 0.0
#
# GPIO Startup
#////////////////////////////////////////////////////////////////
# Input
# Button 4 does not work with gpiozero
# It might work with wiringpi but I do not have driver for A/D converter there
#
#mission = Button(5, pull_up=True, bounce_time=0.2)
#mission.when_pressed = mission_engage
gpio.setmode(gpio.BCM)
gpio.setup(4, gpio.IN)
# Output
# Actuator GPIO14 and GPIO15
#actuator_1=DigitalOutputDevice(pin=14, active_high=True, initial_value=False)
#actuator_2=DigitalOutputDevice(pin=15, active_high=True, initial_value=False)

#
# RTIMU Startup
#////////////////////////////////////////////////////////////////
SETTINGS_FILE = "RTIMULib"
print("Using settings file " + SETTINGS_FILE + ".ini")
if not os.path.exists(SETTINGS_FILE + ".ini"):
  print("Settings file does not exist, will be created")
#
s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)
pressure = RTIMU.RTPressure(s)
humidity = RTIMU.RTHumidity(s)
#
print("IMU Name: " + imu.IMUName())
print("Pressure Name: " + pressure.pressureName())
print("Humidity Name: " + humidity.humidityName())
#
if (not imu.IMUInit()):
    print("IMU Init Failed")
    sys.exit(1)
else:
    print("IMU Init Succeeded");

#
# this is a good time to set any fusion parameters
# 
# The slerp power valule controls the influence of the measured state to correct the predicted state
# 0 = measured state ignored (just gyros), 1 = measured state overrides predicted state.
# In between 0 and 1 mixes the two conditions
#
# You need the gyro! Do not turn it off.
# You should use the Accelerometer. 
#
# If there are strong magnetic anomoalies in pool and from robot
# its better to turn off compass. See code further down.
# I predict the device to keep direction without compass pretty well for 10-20 secs. 
# If you do barrel roles, not sure though.
#
# Also Accelerometer is set to +/- 2g max. It can be set to larger range in ini file. But underwater things go slow.
# 
# The gyroscope bias is updated during runtime when device is not moving. 
# You do not need to worry about not moving the device when its powering on.
# 
imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)
imu.setGyroRunTimeCalibrationEnable(False); # at startup there might be some hickups

if (not pressure.pressureInit()):
    print("Pressure sensor Init Failed")
else:
    print("Pressure sensor Init Succeeded")

if (not humidity.humidityInit()):
    print("Humidity sensor Init Failed")
else:
    print("Humidity sensor Init Succeeded")
#
poll_interval = imu.IMUGetPollInterval()
print("Recommended Poll Interval: %dmS\n" % poll_interval)

#
# ROS Startup
# //////////////////////////////////////////////////////////////////////
pubImu = rospy.Publisher('imu', Imu, queue_size=10)
pubDepth = rospy.Publisher('depth', Float64, queue_size=10)
pubMission = rospy.Publisher('mission', Bool, queue_size=10)
subThrusters = rospy.Subscriber('thrusters', Floats, thruster_callback)
rospy.init_node('rospyrtimulib', anonymous=True)
rate = rospy.Rate(1000 / poll_interval)
#
# read some values, after 2 secs IMU should be stable
i=100
while (i>0):
  i=i-1;
  if imu.IMURead():
    data = imu.getIMUData()
    (data["pressureValid"], data["pressure"], data["pressureTemperatureValid"], data["pressureTemperature"]) = pressure.pressureRead()
    (data["humidityValid"], data["humidity"], data["humidityTemperatureValid"], data["humidityTemperature"]) = humidity.humidityRead()
  # Use rate once ROS is working
  rate.sleep()
  #time.sleep(poll_interval*1.0/1000.0)

#staticpressure = data["pressure"];
fusionPose = data["fusionPose"]
fusionQPose = data["fusionQPose"]
acc = data["accel"]
gyr = data["gyro"]
mag = data["compass"]
pressureTorpedo = 0.0
depth = 0.0
height = 0.0
mission = False
AmpHour = 0.0

#  
# Now turn off magnetometer, assuming to much interference,
# Should be tested if this is necessary by rotationg robot and checking if compass yaw turns properly
# If not, compass needs to be off
#//////////////////////////////////
imu.setCompassEnable(False)
# Enable Gyrobackgroun updates
imu.setGyroRunTimeCalibrationEnable(True); 

#
# ////////////////////////////////////////////////////////////////////////
# Main Loop
# ////////////////////////////////////////////////////////////////////////
#
# Please make sure this loop can be completed in 10ms (100Hz)
# Some IMUs have 100Hz update rate
# SPI A/D might be slow
# 
# User ROS once available
currentTime = rospy.Time.now()
#currentTimeS = currentTime.to_sec()
#previousTimeS = currentTime.to_sec()
currentTimeS = time.time()
initialTime = time.time()
previousTimeMCPS = currentTimeS
previousDisplayTimeS = currentTimeS
previousTimeActuator = currentTimeS

depthPressure = 0
depthData = Float64()
yawInitial = 0
missionStarted = False
rollErr = 0
yawErr = 0
pitchErr = 0
IMUready =  False

# 
while True:
  currentTime  = rospy.Time.now()
  #currentTimeS = currentTime.to_sec()
  currentTimeS = time.time()
  #print("Time %+6.3f" % (currentTimeS))
  # 
  # IMU, humidity and pressure section
  #
  mission = False if gpio.input(4) else True
  
  if imu.IMURead():
    # x, y, z = imu.getFusionData()
    # print("%f %f %f" % (x,y,z))
    data = imu.getIMUData()
    (data["pressureValid"], data["pressure"], data["pressureTemperatureValid"], data["pressureTemperature"]) = pressure.pressureRead()
    (data["humidityValid"], data["humidity"], data["humidityTemperatureValid"], data["humidityTemperature"]) = humidity.humidityRead()
    fusionPose = data["fusionPose"]
    fusionQPose = data["fusionQPose"]
    acc = data["accel"]
    gyr = data["gyro"]
    mag = data["compass"]
    #
    depthPressure = low_pass(data["pressure"], depthPressure, poll_interval/(200.0 + poll_interval))
    depth  = computeDepthSaltWater(depthPressure, staticpressure)
    height = computeHeight(data["pressure"],staticpressure)
    
    # If the IMU is not moving the gyroscope length needs to be smaller than 0.002
    # Otherwise we have issue with gyroscope bias.
    if (not data["motion"]) :
        if ( gyr[0]*gyr[0]+gyr[1]*gyr[1]+gyr[2]*gyr[2] <= 0.000004 ):
            IMUready = True;
       
    #
    # Update ROS IMU settings
    #
    if not rospy.is_shutdown():
        depthData = Float64()
        depthData = depth

        imuData = Imu()
        imuData.orientation.x = fusionQPose[1]
        imuData.orientation.y = fusionQPose[2]
        imuData.orientation.z = fusionQPose[3]
        imuData.orientation.w = fusionQPose[0]
        imuData.angular_velocity.x = gyr[0]
        imuData.angular_velocity.y = gyr[1]
        imuData.angular_velocity.z = gyr[2]
        imuData.linear_acceleration.x = acc[0] * 9.81
        imuData.linear_acceleration.y = acc[1] * 9.81
        imuData.linear_acceleration.z = acc[2] * 9.81
        imuData.header.stamp = currentTime
    #    # no covariance
        imuData.orientation_covariance[0] = -1
        imuData.angular_velocity_covariance[0] = -1
        imuData.linear_acceleration_covariance[0] = -1
    #    #
        imuData.header.frame_id = 'map'
    #    # publish it
        pubImu.publish(imuData)
        pubDepth.publish(depthData)
        pubMission.publish(mission)
    # END OF IMU

  # 
  # At reduced rate write to actuator board
  #
  if ((currentTimeS - previousTimeActuator) > deltaActuator):
    if not mission:
      thrusters = [0 for i in range(10)]

    msg = build_actuator_msg(thrusters)

    ser.write(msg.decode('hex'))

  #        
  # At reduced rate read the MCP3208
  # 
  if ((currentTimeS - previousTimeMCPS) > deltaMCP):
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
      previousTimeS = currentTimeS
      # END OF MCP
    
  #        
  # At reduced rate display data (10Hz)
  # 
  if ((currentTimeS - previousDisplayTimeS) > 0.1):
    if(displayIMU):
      if (data["motion"]):
          print("IMU is moving") 
      else: 
          print("IMU is still")

      if (IMUready): 
          print("Gyro Bias is ok")
      else:
          print("Gyro Bias is not ok")
              
      #print("IMU get data")
      print("Acc x: %+6.2f y: %+6.2f z: %+6.2f length: %+6.3f" % (acc[0], acc[1], acc[2], math.sqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]) ) ) 
      print("Gyr x: %+6.2f y: %+6.2f z: %+6.2f length: %+6.4f" % (gyr[0], gyr[1], gyr[2], math.sqrt(gyr[0]*gyr[0]+gyr[1]*gyr[1]+gyr[2]*gyr[2]) ) )
      print("Mag x: %+6.2f y: %+6.2f z: %+6.2f length: %+6.3f" % (mag[0], mag[1], mag[2], math.sqrt(mag[0]*mag[0]+mag[1]*mag[1]+mag[2]*mag[2]) ) )
      print("Q   x: %+6.2f y: %+6.2f z: %+6.2f w: %+6.2f" % (fusionQPose[1], fusionQPose[2], fusionQPose[3], fusionQPose[0] ) )
      #print("IMU get fusion Pose")
      print("Eul r: %+6.2f p: %+6.2f y: %+6.2f" % (math.degrees(fusionPose[0]), 
          math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
      print("IMU temperature: %+6.1f" % (data["IMUtemperature"]))
      #
      # Depth and Height
      if (data["pressureValid"]):
          print("Pressure: %f, height above sea level: %f, depth below sea level: %f" % (data["pressure"], height, depth))
      if (data["pressureTemperatureValid"]):
          print("Pressure temperature: %+6.3f" % (data["pressureTemperature"]))
      #
      # Humidity
      if (data["humidityValid"]):
          print("Humidity: %+6.1f" % (data["humidity"]))
      if (data["humidityTemperatureValid"]):
          print("Humidity temperature: %+6.1f" % (data["humidityTemperature"]))
      print("Torpedo Pressure Reading = %2.2f psi" % (pressureTorpedo) )
      print("AmpHour consumed %3.3f [Ahr]" % (AmpHour) )
      print("Depth: %3.3f" % (depth))
      print("Static Pressure: %6.3f" % (staticpressure))
      print "Depth: " + str(depthData)
      print "Roll Error: " + str(rollErr)
      print "Pitch Error: " + str(pitchErr)
      print "Yaw Error: " + str(yawErr)
    previousDisplayTimeS = currentTimeS
  
  #
  # Here need to read message from ROS master and act accordingly on outputs
  #
  # actuator_1.on();
  # actuator_2.on();
  # actuator_1.off();
  # actuator_2.off();

  # release task
  # use rate once ROS is working and booted up
  rate.sleep()
  timeRemaining = poll_interval/1000.0 - (time.time() - currentTimeS)
  if (timeRemaining > 0):
    time.sleep(timeRemaining)

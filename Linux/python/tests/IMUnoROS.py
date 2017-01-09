#!/usr/bin/env python
#/////////////////////////////////////////////////////////////
#
# ROS related imports
#import rospy
#from sensor_msgs.msg import Imu
#from std_msgs.msg import Float64
#from std_msgs.msg import Bool
#from rospy_tutorials.msg import Floats

# RTIMU reltate imports
import RTIMU

# Other Imports
import sys
import os.path
import math
import time
import datetime

sys.path.append('.')

displayIMU = True
staticPressure = 1012.0

#
# General functions
#//////////////////////////////////////////////////////////
def low_pass(curr, prev, alpha):
  return curr * alpha + prev * (1 - alpha)

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

def computeHeight(pressure, staticPressure):
    if (staticPressure > 0):
      return 44330.8 * (1 - pow(pressure / staticPressure, 0.190263))
    else:
      return 0.0
  
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

def computeDepthFreshWater(pressure, staticPressure):
    return (pressure - staticPressure) * 0.01019716

def computeDepthSaltWater(pressure, staticPressure):
    p=(pressure-staticPressure)/100
    return ((( (-1.82e-15*p + 2.279e-10 )*p - 2.2512e-5 )*p + 9.72659) * p) / 9.7954

#
#/////////////////////////////////////////////////////////////////
# Startup 
#/////////////////////////////////////////////////////////////////
#

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
    print("IMU Init Succeeded")

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
# at startup there might be some hickups:
imu.setGyroRunTimeCalibrationEnable(False) 

if (not pressure.pressureInit()):
    print("Pressure sensor Init Failed")
else:
    print("Pressure sensor Init Succeeded")

if (not humidity.humidityInit()):
    print("Humidity sensor Init Failed")
else:
    print("Humidity sensor Init Succeeded")
#
IMU_poll_interval = imu.IMUGetPollInterval()*1.0/1000.0
print("Recommended Poll Interval for IMU: %4.3fS" % IMU_poll_interval)
pressure_poll_interval = pressure.pressureGetPollInterval()*1.0/1000.0
print("Recommended Poll Interval for pressure sensor: %4.3fS" % pressure_poll_interval)
humidity_poll_interval = humidity.humidityGetPollInterval()*1.0/1000.0
print("Recommended Poll Interval for humidity sensor: %4.3fS" % humidity_poll_interval)

#
# read some values, after 1-2 secs IMU should be stable and static pressure measured

print("Dry Run")

i=int(2.0/IMU_poll_interval)
firsttime = True
while (i>0):
  i -= 1
  if imu.IMURead():
    dataIMU = imu.getIMUData()
    
  if pressure.pressureRead():
    dataPressure = pressure.getPressureData()

  if humidity.humidityRead():     
    dataHumidity = humidity.getHumidityData()
    
    if dataPressure["pressureValid"] :
      if dataPressure["pressure"] > 500.0 and dataPressure["pressure"] < 1500.0  :
        if firsttime :
          staticPressure = dataPressure["pressure"]
          firsttime = False
        else :
          staticPressure = low_pass(dataPressure["pressure"], staticPressure, 0.1)
    # print("Pressure: %f, Static: %f" % (dataPressure["pressure"], staticPressure))

  # Use rate class to accurately time the loop
  # rate.sleep()
  time.sleep(IMU_poll_interval)

#
# Initialize values
fusionPose = dataIMU["fusionPose"]
fusionQPose = dataIMU["fusionQPose"]
acc = dataIMU["accel"]
gyr = dataIMU["gyro"]
mag = dataIMU["compass"]

#  
# Now turn off magnetometer, assuming to much interference,
# Should be tested if this is necessary by rotationg robot and checking if compass yaw turns properly
# If not, compass needs to be off
#//////////////////////////////////
imu.setCompassEnable(False)
# Enable Gyrobackgroun updates
imu.setGyroRunTimeCalibrationEnable(True)

print("Initialization completed")

#
# ////////////////////////////////////////////////////////////////////////
# Main Loop
# ////////////////////////////////////////////////////////////////////////
#
# Please make sure this loop can be completed in 10ms (100Hz)
# Some IMUs have 100Hz update rate
# 
currentTimeS = time.time()
previousDisplayTimeS = currentTimeS 

IMUready =  False

lastHumidityPoll = time.time()
lastPressurePoll = time.time()
lastIMUPoll = time.time()
previousRateTime = time.time()

depthPressure = staticPressure
depth  = 0.0
height = 0.0
pressureFiltered = staticPressure

humidityFiltered = dataHumidity["humidity"]

#print(acc)
#print(gyr)
#print(mag)

acc=(0.0, 0.0, 0.0)
gyr=(0.0, 0.0, 0.0)
mag=(0.0, 0.0, 0.0)

fusionQPose = (0.0, 0.0, 0.0, 0.0)
fusionPose = (0.0, 0.0, 0.0, 0.0)

IMUCounter =0
pressureCounter=0
humidityCounter=0
IMURate=0
humidityRate=0
pressureRate=0

# 
while True:
  currentTimeS = time.time()
  #print("Time %+6.3f" % (currentTimeS))
  
  # 
  # IMU
  #
  
  # do we have IMU stalled ?
  if (currentTimeS - lastIMUPoll) > 50.0*IMU_poll_interval:
    printf("IMU stalled!!!!!!!!!!!!")
    imu.IMUInit()
  
  if currentTimeS - lastIMUPoll >= IMU_poll_interval :
    if imu.IMURead():
      dataIMU = imu.getIMUData()
      lastIMUPoll = currentTimeS
      IMUCounter +=1
      if dataIMU["accelValid"]:
        acc = dataIMU["accel"]
      if dataIMU["gyroValid"]:
        gyr = dataIMU["gyro"]
      if dataIMU["compassValid"]:
        mag = dataIMU["compass"]
  
      # data in expected range ?
      if ( (gyr[0]*gyr[0]+gyr[1]*gyr[1]+gyr[2]*gyr[2]) >= 35*35 ) or ( (acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]) >= 16*16 ) or ( (mag[0]*mag[0]+mag[1]*mag[1]+mag[2]*mag[2]) >= 1000*1000) :
        imu.IMUInit()
      else:
        if dataIMU["fusionPoseValid"] :
          fusionPose = dataIMU["fusionPose"]
        if dataIMU["fusionQPoseValid"] :
          fusionQPose = dataIMU["fusionQPose"]
  
      # If the IMU is not moving the gyroscope length needs to be smaller than 0.002
      # Otherwise we have issue with gyroscope bias.
      # Assume if gyro bias was ok once it will stay that way
      if ( (IMUready == False) and (dataIMU["motion"] == False) ):
        if ( gyr[0]*gyr[0]+gyr[1]*gyr[1]+gyr[2]*gyr[2] <= 0.000004 ):
          IMUready = True
          
  # 
  # Pressure
  #

  if currentTimeS - lastPressurePoll >= pressure_poll_interval :
    if pressure.pressureRead():
      lastPressurePoll = currentTimeS
      dataPressure = pressure.getPressureData()
      pressureCounter +=1
      pressureFiltered = low_pass(dataPressure["pressure"], pressureFiltered, 0.1)
      depth  = computeDepthSaltWater(pressureFiltered, staticPressure)
      height = computeHeight(pressureFiltered,staticPressure)

  # 
  # Humidity
  #
      
  if currentTimeS - lastHumidityPoll >= humidity_poll_interval :
    if humidity.humidityRead():
      lastHumidityPoll = currentTimeS
      dataHumidity = humidity.getHumidityData()
      humidityCounter +=1
      humidityFiltered = low_pass(dataHumidity["humidity"], humidityFiltered, 0.1)

  #        
  # Compute Data Rates
  # 

  if ((currentTimeS - previousRateTime) >= 1.0):
    IMURate= IMUCounter
    pressureRate = pressureCounter
    humidityRate = humidityCounter
    pressureCounter = 0
    humidityCounter = 0
    IMUCounter = 0
    previousRateTime = currentTimeS
     
  #        
  # At reduced rate display data (10Hz)
  # 

  if ((currentTimeS - previousDisplayTimeS) > 0.1):
    if(displayIMU):
      if (dataIMU["motion"]):
        print("IMU is moving") 
      else: 
        print("IMU is still")

      if (IMUready): 
        print("Gyro Bias is ok")
      else:
        print("Gyro Bias is not ok")

      print("IMU rate: %d pressure rate: %d humidity rate: %d" % (IMURate, pressureRate, humidityRate) ) 
              
      #print("IMU get data")
      print("Acc x: %+6.3f y: %+6.3f z: %+6.3f length: %+6.3f" % (acc[0], acc[1], acc[2], math.sqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]) ) ) 
      print("Gyr x: %+6.3f y: %+6.3f z: %+6.3f length: %+6.4f" % (gyr[0], gyr[1], gyr[2], math.sqrt(gyr[0]*gyr[0]+gyr[1]*gyr[1]+gyr[2]*gyr[2]) ) )
      print("Mag x: %+6.2f y: %+6.2f z: %+6.2f length: %+6.3f" % (mag[0], mag[1], mag[2], math.sqrt(mag[0]*mag[0]+mag[1]*mag[1]+mag[2]*mag[2]) ) )
      print("Q   x: %+6.2f y: %+6.2f z: %+6.2f w: %+6.2f" % (fusionQPose[1], fusionQPose[2], fusionQPose[3], fusionQPose[0] ) )
      #print("IMU get fusion Pose")
      print("Eul r: %+6.2f p: %+6.2f y: %+6.2f" % (math.degrees(fusionPose[0]), 
        math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
      print("IMU temperature: %+6.1f" % (dataIMU["temperature"]))
      #
      # Depth and Height
      if (dataPressure["pressureValid"]):
        print("Pressure: %+6.1f, %+6.1f, height above sea level: %+6.1f, depth below sea level: %+6.2f" % (dataPressure["pressure"], pressureFiltered, height, depth))
      print("Static Pressure: %6.3f" % (staticPressure))
      if (dataPressure["temperatureValid"]):
        print("Pressure temperature: %+6.3f" % (dataPressure["temperature"]))        
      #
      # Humidity
      if (dataHumidity["humidityValid"]):
        print("Humidity: %+6.1f, %+6.1f" % (dataHumidity["humidity"],humidityFiltered))
      if (dataHumidity["temperatureValid"]):
        print("Humidity temperature: %+6.1f" % (dataHumidity["temperature"]))
    
    previousDisplayTimeS = currentTimeS

  #
  # release task
  #
  
  timeRemaining = 0.001 - (time.time() - currentTimeS)
  if (timeRemaining > 0):
    time.sleep(timeRemaining)

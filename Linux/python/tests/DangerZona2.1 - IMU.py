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
      return 44330.8 * (1 - pow(pressure / staticPressure, 0.190263));
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

def computeDepthFreshWater(pressure, staticPressure):
    return (pressure - staticPressure) * 0.01019716;    

def computeDepthSaltWater(pressure, staticPressure):
    p=(pressure-staticPressure)/100
    return ((( (-1.82e-15*p + 2.279e-10 )*p - 2.2512e-5 )*p + 9.72659) * p) / 9.7954;    

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
rospy.init_node('rospyrtimulib', anonymous=True)

rate = rospy.Rate(1000 / poll_interval)

#
# read some values, after 1-2 secs IMU should be stable and static pressure measured
i=100
while (i>0):
  i=i-1;
  if imu.IMURead():
    data = imu.getIMUData()
    (data["pressureValid"], data["pressure"], data["pressureTemperatureValid"], data["pressureTemperature"]) = pressure.pressureRead()
    (data["humidityValid"], data["humidity"], data["humidityTemperatureValid"], data["humidityTemperature"]) = humidity.humidityRead()
    staticPressure = low_pass(data["pressure"], staticPressure, poll_interval/(200.0 + poll_interval))
  # Use rate class to accuratly time the loop
  rate.sleep()
  #time.sleep(poll_interval*1.0/1000.0)

#
# Initialize values
fusionPose = data["fusionPose"]
fusionQPose = data["fusionQPose"]
acc = data["accel"]
gyr = data["gyro"]
mag = data["compass"]

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
# 
currentTimeS = time.time()
previousDisplayTimeS = 

depthPressure = 0.0
depthData = Float64()
yawInitial = 0.0
missionStarted = False
rollErr = 0.0
yawErr = 0.0
pitchErr = 0.0
depth = 0.0
height = 0.0
IMUready =  False

# 
while True:
  currentTimeS = time.time()
  #print("Time %+6.3f" % (currentTimeS))
  
  # 
  # IMU, humidity and pressure section
  #
  
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
    depth  = computeDepthSaltWater(depthPressure, staticPressure)
    height = computeHeight(data["pressure"],staticPressure)
    
    # If the IMU is not moving the gyroscope length needs to be smaller than 0.002
    # Otherwise we have issue with gyroscope bias.
  # Assume if gyro bias was ok once it will stay that way
    if ( (IMUready == False) && (not data["motion"]) ) :
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
        imuData.header.stamp = rospy.Time.now()
    
    # no covariance
        imuData.orientation_covariance[0] = -1
        imuData.angular_velocity_covariance[0] = -1
        imuData.linear_acceleration_covariance[0] = -1
   
        imuData.header.frame_id = 'map'
    
    # publish it
        pubImu.publish(imuData)
        pubDepth.publish(depthData)
    # END OF IMU

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

    print("Depth: %3.3f" % (depth))
      print("Static Pressure: %6.3f" % (staticPressure))
      print "Depth: " + str(depthData)
      print "Roll Error: " + str(rollErr)
      print "Pitch Error: " + str(pitchErr)
      print "Yaw Error: " + str(yawErr)
    
    previousDisplayTimeS = currentTimeS

  # release task
  # use rate once ROS is working and booted up
  rate.sleep()
 
  # timeRemaining = poll_interval/1000.0 - (time.time() - currentTimeS)
  # if (timeRemaining > 0):
  #  time.sleep(timeRemaining)

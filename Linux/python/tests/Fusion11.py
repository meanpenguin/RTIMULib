import sys

sys.path.append('.')
import RTIMU
import os.path
import time
import math

SETTINGS_FILE = "RTIMULib"

#  computeHeight() - the conversion uses the formula:
#
#  h = (T0 / L0) * ((p / P0)**(-(R* * L0) / (g0 * M)) - 1)
#
#  where:
#  h  = height above sea level
#  T0 = standard temperature at sea level = 288.15
#  L0 = standard temperatur elapse rate = -0.0065
#  p  = measured pressure
#  P0 = static pressure = 1013.25
#  g0 = gravitational acceleration = 9.80665
#  M  = mloecular mass of earth's air = 0.0289644
#  R* = universal gas constant = 8.31432
#
#  Given the constants, this works out to:
#
#  h = 44330.8 * (1 - (p / P0)**0.190263)

def computeHeight(pressure):
    return 44330.8 * (1 - pow(pressure / 1013.25, 0.190263));
    
#
# Saltwater density is 1.025 kg/l
# Freshwater density is 1.0 kg/l
# Gravity is 9.80665 m/s/s
# delta P = rho g h
# see level pressure in average: 1013.25
# 1 millibar = 100 pascal [kg/m/s/s]
# 100/9.8/1.025
#
# http://www.seabird.com/document/an69-conversion-pressure-depth
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

def computeDepthFreshWater(pressure, staticpressure):
    return (pressure - staticpressure) * 0.01019716;	

def computeDepthSaltWater(pressure, staticpressure):
    p=(pressure-staticpressure)/100
    return ((( (-1.82e-15*p + 2.279e-10 )*p - 2.2512e-5 )*p + 9.72659) * p) / 9.7954;	
	
print("Using settings file " + SETTINGS_FILE + ".ini")
if not os.path.exists(SETTINGS_FILE + ".ini"):
  print("Settings file does not exist, will be created")

s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)
pressure = RTIMU.RTPressure(s)
humidity = RTIMU.RTHumidity(s)

print("IMU Name: " + imu.IMUName())
print("Pressure Name: " + pressure.pressureName())
print("Humidity Name: " + humidity.humidityName())

if (not imu.IMUInit()):
    print("IMU Init Failed")
    sys.exit(1)
else:
    print("IMU Init Succeeded");

# this is a good time to set any fusion parameters

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

poll_interval = imu.IMUGetPollInterval()
print("Recommended Poll Interval: %dmS\n" % poll_interval)

# Dry run 
i = 0;
while (i < 80): 
	if  imu.IMURead(): 
		i = i + 1;
		
# Set conditions for continous operation
data = imu.getIMUData()
staticpressure=data["pressure"];
imu.setGyroRunTimeCalibrationEnable(True);
  
while True:
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
    if (data["motion"]):
        print("IMU is moving") 
    else: 
        print("IMU is still")
    #print("IMU get data")
    print("Acc x: %+6.2f y: %+6.2f z: %+6.2f length: %+6.3f" % (acc[0], acc[1], acc[2], math.sqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]) ) ) 
    print("Gyr x: %+6.2f y: %+6.2f z: %+6.2f length: %+6.4f" % (gyr[0], gyr[1], gyr[2], math.sqrt(gyr[0]*gyr[0]+gyr[1]*gyr[1]+gyr[2]*gyr[2]) ) )
    print("Mag x: %+6.2f y: %+6.2f z: %+6.2f length: %+6.3f" % (mag[0], mag[1], mag[2], math.sqrt(mag[0]*mag[0]+mag[1]*mag[1]+mag[2]*mag[2]) ) )
    #print("IMU get fusion Pose")
    print("Q   x: %+6.2f y: %+6.2f z: %+6.2f w: %+6.2f" % (fusionQPose[1], fusionQPose[2], fusionQPose[3], fusionQPose[0] ) )
    print("Eul r: %+6.2f p: %+6.2f y: %+6.2f" % (math.degrees(fusionPose[0]), 
        math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
    print("IMU temperature: %+6.1f" % (data["IMUtemperature"]))
    if (data["pressureValid"]):
        print("Pressure: %f, height above sea level: %f, depth below sea level: %f" % (data["pressure"], computeHeight(data["pressure"],staticpressure), computeDepthSaltWater(data["pressure"],staticpressure)))
    if (data["pressureTemperatureValid"]):
        print("Pressure temperature: %+6.1f" % (data["pressureTemperature"]))
    if (data["humidityValid"]):
        print("Humidity: %+6.1f" % (data["humidity"]))
    if (data["humidityTemperatureValid"]):
        print("Humidity temperature: %+6.1f" % (data["humidityTemperature"]))
    time.sleep(poll_interval*1.0/1000.0)

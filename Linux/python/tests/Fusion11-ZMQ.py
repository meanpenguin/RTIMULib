import sys, getopt

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
    
def computeDepth(pressure):
    return (pressure - 1013.25) * 0.01019716;	

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

print("Fusion Type: " + s.FusionType)

while True:
  if imu.IMURead():
    # x, y, z = imu.getFusionData()
    # print("%f %f %f" % (x,y,z))
    data = imu.getIMUData()
    (data["pressureValid"], data["pressure"], data["pressureTemperatureValid"], data["pressureTemperature"]) = pressure.pressureRead()
    (data["humidityValid"], data["humidity"], data["humidityTemperatureValid"], data["humidityTemperature"]) = humidity.humidityRead()
    fusionPose = data["fusionPose"]
    fusionQPose = data["fusionQPose"]
    accel = data["accel"]
    gyro = data["gyro"]
    mag = data["compass"]
	
    print("r: %f p: %f y: %f" % (math.degrees(fusionPose[0]), 
        math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
    print("S: %f x: %f y: %f z: %f" % ((fusionQPose[0]), 
        (fusionQPose[1]), (fusionQPose[2]), (fusionQPose[3])))
    print("Accel x: %f y: %f z: %f" % ((accel[0]), 
        (accel[1]), (accel[2])))
    print("Gyro  x: %f y: %f z: %f" % ((gyro[0]), 
        (gyro[1]), (gyro[2])))
    print("Mag   x: %f y: %f z: %f" % ((mag[0]), 
        (mag[1]), (mag[2])))

    if (data["pressureValid"]):
        print("Pressure: %f, height above sea level: %f, depth below sea level: %f" % (data["pressure"], computeHeight(data["pressure"]), computeDepth(data["pressure"])))
    if (data["pressureTemperatureValid"]):
        print("Pressure temperature: %f" % (data["pressureTemperature"]))
    if (data["humidityValid"]):
        print("Humidity: %f" % (data["humidity"]))
    if (data["humidityTemperatureValid"]):
        print("Humidity temperature: %f" % (data["humidityTemperature"]))
    time.sleep(poll_interval*1.0/1000.0)

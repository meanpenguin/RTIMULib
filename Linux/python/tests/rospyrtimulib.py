#!/usr/bin/env python
#/////////////////////////////////////////////////////////////////////////////
#//
#// Software License Agreement (BSD License)
#//
#// Copyright (c) 2015, richards-tech, LLC.
#//
#// Redistribution and use in source and binary forms, with or without
#// modification, are permitted provided that the following conditions are met:
#//
#//   * Redistributions of source code must retain the above copyright
#//     notice, this list of conditions and the following disclaimer.
#//   * Redistributions in binary form must reproduce the above copyright
#//     notice, this list of conditions and the following disclaimer in the
#//     documentation and/or other materials provided with the distribution.
#//   * Neither the name of richards-tech, LLC nor the
#//     names of its contributors may be used to endorse or promote products
#//     derived from this software without specific prior written permission.
#//
#// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
#// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
#// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# ROS related imports

import rospy
from sensor_msgs.msg import Imu

# RTIMULib related imports

import RTIMU
import os.path
import sys

SETTINGS_FILE = "RTIMULib"

def rospyrtimulib():
    imuData = Imu()
    print("Using settings file " + SETTINGS_FILE + ".ini")
    if not os.path.exists(SETTINGS_FILE + ".ini"):
        print("Settings file does not exist, will be created")

    s = RTIMU.Settings(SETTINGS_FILE)
    imu = RTIMU.RTIMU(s)

    print("IMU Name: " + imu.IMUName())

    if (not imu.IMUInit()):
        print("IMU Init Failed")
        sys.exit(1)
    else:
        print("IMU Init Succeeded")

    # this is a good time to set any fusion parameters

    imu.setSlerpPower(0.02)
    imu.setGyroEnable(True)
    imu.setAccelEnable(True)
    imu.setCompassEnable(True)

    pub = rospy.Publisher('imu', Imu, queue_size=10)
    rospy.init_node('rospyrtimulib', anonymous=True)
    rate = rospy.Rate(1000 / imu.IMUGetPollInterval())

    while not rospy.is_shutdown():
        if imu.IMURead():
            # collect the data
            data = imu.getIMUData()

            fusionQPose = data["fusionQPose"]
            imuData.orientation.x = fusionQPose[1]
            imuData.orientation.y = fusionQPose[2]
            imuData.orientation.z = fusionQPose[3]
            imuData.orientation.w = fusionQPose[0]
 
            gyro = data["gyro"]
            imuData.angular_velocity.x = gyro[0]
            imuData.angular_velocity.y = gyro[1]
            imuData.angular_velocity.z = gyro[2]

            accel = data["accel"]
            imuData.linear_acceleration.x = accel[0] * 9.81
            imuData.linear_acceleration.y = accel[1] * 9.81
            imuData.linear_acceleration.z = accel[2] * 9.81
            imuData.header.stamp = rospy.Time.now()

            # no covariance
            imuData.orientation_covariance[0] = -1
            imuData.angular_velocity_covariance[0] = -1
            imuData.linear_acceleration_covariance[0] = -1

            imuData.header.frame_id = 'map'

            # publish it
            pub.publish(imuData)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospyrtimulib()
    except rospy.ROSInterruptException:
        pass

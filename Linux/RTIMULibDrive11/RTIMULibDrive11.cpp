////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "RTIMULib.h"

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include <termios.h>
#include <unistd.h>
#include <ctype.h>
#include <sys/wait.h>
#include <sys/ioctl.h>

char   getUserChar();

static RTIMUSettings *settings;
static RTIMU         *imu;
static RTPressure    *pressure; 
static RTHumidity    *humidity;
static RTMotion      *motion;


int main()
{
    int IMUsampleCount = 0;
    int IMUsampleRate = 0;
    int pressureSampleCount = 0;
    int pressureSampleRate = 0;
    int humiditySampleCount = 0;
    int humiditySampleRate = 0;
    int timeout;
    uint64_t rateTimer;
    uint64_t now;
    uint64_t lastPoll;
    uint64_t lastIMUPoll;
    uint64_t lastPressurePoll;
    uint64_t lastHumidityPoll;
    uint64_t lastDisplayPoll;
    uint64_t lastUserPoll;
    
    int IMUpollInterval = 1;
	int pressurePollInterval = 1;
	int humidityPollInterval = 1;

    bool ismoving = false;
    bool enableCompass = true;
    bool displayData = true;
    char keystatus[26] = " ------M-G->----------- \n";
    char sysstatus1[64] = " -------------------------------------------- \n";
    char sysstatus2[64] = " -------------------------------------------- \n";
    
    RTVector3 residuals;
    RTFLOAT heading;
    float heading_avg = 0.0f;
    float humidity_avg = 0.0f;
    float pressure_avg = 0.0f;
    char input;
    bool mustExit = false;
    
    RTIMU_DATA imuData;
    HUMIDITY_DATA humidityData;
    PRESSURE_DATA pressureData;
    MOTION_DATA motionData;

    imuData.fusionPoseValid = false;
    imuData.fusionQPoseValid = false;
    imuData.gyroValid = false;
    imuData.accelValid = false;
    imuData.compassValid = false;
    imuData.motion = true;
    imuData.temperatureValid = false;
    imuData.temperature = -9999.9999;

    pressureData.pressureValid = false;
    pressureData.pressure = -9999.9999;
    pressureData.temperatureValid = false;
    pressureData.temperature = -9999.9999;

    humidityData.humidityValid = false;
    humidityData.humidity = -9999.9999;
    humidityData.temperatureValid = false;
    humidityData.temperature = -9999.9999;

    //  using RTIMULib here allows it to use the .ini file generated by RTIMULibDemo.

    settings = new RTIMUSettings("RTIMULib");

    printf("Creating IMU\n");
    imu = RTIMU::createIMU(settings);
    if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)) {
        printf("No IMU found\n");
        exit(1);
    }

    //  set up IMU
    printf("Initializing IMU\n");
    if (!imu->IMUInit()) { exit(1);}

    //  This is a convenient place to change fusion parameters
    imu->setGyroRunTimeCalibrationEnable(false);             // turn off gyro bias calibration at startup, allow system to equiblirate first
    imu->setSlerpPower(0.02);
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(false);
    imu->setDebugEnable(false); // turn on if you want to see Fusion information

    printf("Getting Poll Interval\n");
    IMUpollInterval = imu->IMUGetPollInterval() * 1000;

    printf("Creating Motion System\n");
    motion = new  RTMotion(settings);
    motion->motionInit();

    //  set up pressure sensor

    printf("Creating Pressure\n");
    pressure = RTPressure::createPressure(settings);

    RTFLOAT staticPressure = 1013.25f;

    if (pressure != NULL) {
        pressure->pressureInit();
        pressurePollInterval = pressure->pressureGetPollInterval() * 1000;
    }

    //  set up humidity sensor

    printf("Creating Humidity\n");
    humidity = RTHumidity::createHumidity(settings);

    if (humidity != NULL) {
        humidity->humidityInit();
        humidityPollInterval = humidity->humidityGetPollInterval() * 1000;
    }
    
    //  set up console io

    struct termios    ctty;
    tcgetattr(fileno(stdout), &ctty);
    ctty.c_lflag &= ~(ICANON);
    tcsetattr(fileno(stdout), TCSANOW, &ctty);

    // dry run of the system to stabilize
    printf("Dry Run\n");

    int i=0;
    lastPoll = RTMath::currentUSecsSinceEpoch();
    while (i < 80) {

        now = RTMath::currentUSecsSinceEpoch();
        int pollDelay = (IMUpollInterval - (int)(now - lastPoll));
        if (pollDelay > 0) { usleep(pollDelay); }
        lastPoll = now;

        if  (imu->IMURead()) { i++; }
        if (humidity != NULL) { humidity->humidityRead(); }
        if (pressure != NULL) { pressure->pressureRead(); }
    }

    
    pressureData = pressure->getPressureData();
    if (pressureData.pressureValid) {staticPressure = pressureData.pressure;}

    imu->setGyroRunTimeCalibrationEnable(true);     // enable background gyro calibration

    //  set up timers

    rateTimer = lastDisplayPoll = lastPoll = lastIMUPoll = lastHumidityPoll = lastPressurePoll = lastUserPoll = RTMath::currentUSecsSinceEpoch();
    timeout= 50*IMUpollInterval;

    printf("System ready\n");

    //  now just process data
    while (!mustExit) {
   
        //  run this loop no faster than 1000Hz
        now = RTMath::currentUSecsSinceEpoch();
        int pollDelay = (1000 - (int)(now - lastPoll));
        if (pollDelay > 0) { usleep(pollDelay); }
        lastPoll = now;

        // check IMU stalled
        if ( (int)(now - lastIMUPoll) > timeout ) {
            // We have IMU stalled and need to reset it
            HAL_ERROR("!!!!!!!!!!!!!!!!!!!! IMU RESET: waiting for data for too long !!!!!!!!!!!!!!!!!!!!\n");
            imu->IMUInit(); // Its not yet tested if this call will pull IMU out of stall
            lastPoll = lastIMUPoll = now = RTMath::currentUSecsSinceEpoch();
        }
            
        now = RTMath::currentUSecsSinceEpoch();
        if ( (int)(now - lastIMUPoll) >= IMUpollInterval ) {
			while (imu->IMURead()) {
				// set loop time stamp
				now = lastIMUPoll = RTMath::currentUSecsSinceEpoch();
				imuData=imu->getIMUData();

				// check data in rcd ange
				if ( (imuData.gyro.length() > 35.0) || (imuData.accel.length() > 16.0) || (imuData.compass.length() > 1000.0) ) {
					// IMU Data Error
					HAL_ERROR("!!!!!!!!!!!!!!!!!!!! IMU RESET: data out of range !!!!!!!!!!!!!!!!!!!!\n");
					imu->IMUInit(); // Its not yet tested if this call will properly reset IMU
					lastIMUPoll = RTMath::currentUSecsSinceEpoch();
					break;
				}

				// Motion detection
				ismoving=motion->detectMotion(imuData.accel, imuData.gyro);

				// Residuals of Acceleration minus Gravity
				residuals = imu->getAccelResiduals();

				// Tilt Compensated Heading
				heading     = imuData.fusionPose.toHeading(imuData.compass, settings->m_compassAdjDeclination);
				heading_avg = motion->updateAverageHeading(heading); // smooth it out to approx. 10Hz update rate

				// Attempt velocity and position estimation
				motion->updateVelocityPosition(residuals, imuData.fusionQPose, 9.81f, imuData.timestamp, ismoving);
				// checking rotations: motion->updateVelocityPosition(imuData.accel, imuData.fusionQPose, 9.81f, imuData.timestamp, ismoving);
				motionData = motion->getMotionData();

				//  update rate every second
				IMUsampleCount++;
			} // end check for new IMU data
        }

        //  read pressure data 
        if (pressure != NULL) {
          now = RTMath::currentUSecsSinceEpoch();
          if ( (int)(now - lastPressurePoll) >= pressurePollInterval ) { // need 4.5 ms for a pressure or temperature conversion on BMP180
            // pressure read requires multiple calls in most sensors: 
            //   First call initiates conversion, 
            //   Second call reads temperature, and initiates pressure conversion
            //   Third call reads pressure and then calibrates the reading
            lastPressurePoll = now;
            if (pressure->pressureRead()){
                pressureData=pressure->getPressureData();
                if (pressureData.pressureValid) {
                  pressure_avg = pressure->updateAveragePressure(pressureData.pressure); // smooth it out
                }
	            pressureSampleCount++;
            };
          }
        }
        
        //  read the humidity data
        if (humidity != NULL) {
          now = RTMath::currentUSecsSinceEpoch();
          if ( (int)(now - lastHumidityPoll) >= humidityPollInterval ) { // 80ms for a combined temperature & humidity conversion on HTS221
            lastHumidityPoll = now;
            if (humidity->humidityRead()) {
                humidityData = humidity->getHumidityData();
                if (humidityData.humidityValid) {
                  humidity_avg = humidity->updateAverageHumidity(humidityData.humidity); // smooth it out
                }
	            humiditySampleCount++;
            }
          }
        }

        if ((now - rateTimer) > 1000000) {
            IMUsampleRate = IMUsampleCount;
            IMUsampleCount = 0;
            pressureSampleRate = pressureSampleCount;
            pressureSampleCount = 0;
            humiditySampleRate = humiditySampleCount;
            humiditySampleCount = 0;
            rateTimer = now;
        }

        //  display 10 times per second
        now = RTMath::currentUSecsSinceEpoch();
        if ((now - lastDisplayPoll) > 100000) {
            lastDisplayPoll = now;
            if (displayData) {
                printf("\e[1;1H\e[2J");  // clear screen and move cursor 1/1

                printf("RTIMU: Fusion %s, Compass is %s", RTFusion::fusionName(settings->m_fusionType), enableCompass ? "On\n" : "Off\n" );
                printf("Sample Rate IMU:%d, p:%d, h:%d\n", IMUsampleRate, pressureSampleRate, humiditySampleRate);
                printf("Heading:%4.1f Heading Ave:%4.1f\n", RTMATH_RAD_TO_DEGREE * heading, RTMATH_RAD_TO_DEGREE * heading_avg);
                printf(ismoving ? "IMU is moving\n" : "IMU is still\n");
                printf("%s", RTMath::display("Quaternion", imuData.fusionQPose));
                printf("%s", RTMath::displayDegrees("Pose ", imuData.fusionPose));
                printf("%s", RTMath::displayRadians("Accel", imuData.accel));
                printf("%s", RTMath::displayRadians("Gyro ", imuData.gyro));
                printf("%s", RTMath::displayRadians("Mag  ", imuData.compass));
                printf("%s", RTMath::displayRadians("Residuals     ", residuals));
                RTVector3 residualsBias = motion->getResidualsBias();
                printf("%s", RTMath::displayRadians("Residuals Bias", residualsBias));

                if (pressure != NULL) {
                    printf("Pressure:%4.2f, Avg:%4.2f, Height above sea level: %4.1f, Depth below sea level:%4.3f\n",
                           pressureData.pressure, pressure_avg, RTMath::convertPressureToHeight(pressure_avg, staticPressure),
                                                  RTMath::convertPressureToDepth(pressure_avg, staticPressure));
                }
                if (humidity != NULL) {
                    printf("Humidity:%4.1f%% avg:%4.1f%%\n",
                           humidityData.humidity, humidity_avg);
                }

                printf("Temperature: IMU:%4.1f", imuData.temperature);
                if (pressure != NULL) { printf(", Pressure Sensor:%4.2f", pressureData.temperature); }
                if (humidity != NULL) { printf(", Humidity Sensor:%4.2f", humidityData.temperature); }
                printf("\n");

                printf("%s", RTMath::displayRadians("World Accel   ",       motionData.worldAcceleration));
                printf("%s", RTMath::displayRadians("World Velocity",       motionData.worldVelocity));
                printf("%s", RTMath::displayRadians("World Velocity Drift", motionData.worldVelocityDrift));
                printf("%s", RTMath::displayRadians("World Position",       motionData.worldPosition));
                printf("%s", RTMath::displayRadians("Local Residuals",      motionData.residuals));

                printf("Timestamp: %" PRIu64 "\n", imuData.timestamp);
                printf(" a-p-z-m-M-x----------- \n");
                printf("%s", keystatus);
                // printf("%s", sysstatus1);
                // printf("%s", sysstatus2);

                printf("%s", "a: runtm Acc cal, A: disp max/min, p: zero stat press, z: zero\n");
                printf("%s", "m/M: Mag off/on, g/G: Gyro runtm cal off/on \n");
                printf("%s", "s/S: Turn streaming on/OFF\n");
                printf("%s", "d/D: Gyro manual bias off/on, x: exit\n");

                fflush(stdout);
            } // display data
        }

        now = RTMath::currentUSecsSinceEpoch();
        if ( (now - lastUserPoll) > 500000 ) { // 20Hz
            lastUserPoll = now;
            if ((input = getUserChar()) != 0) {
                switch (input) {
                    case 'a' :
                        // conduct Accelerometer Max/Min calibration
                        keystatus[1] ='A';
                        keystatus[3] ='-';
                        keystatus[5] ='-';
                        imu->runtimeAdjustAccelCal();
                        break;
                    case 'A' :
                        keystatus[1] ='-';
                        sprintf(sysstatus1, "%s", RTMath::displayRadians("Acc Cal Max ", settings->m_accelCalMax));
                        sprintf(sysstatus2, "%s", RTMath::displayRadians("Acc Cal Min ", settings->m_accelCalMin));
                        break;
                    case 'p' :
                        keystatus[1] ='-';
                        keystatus[3] ='.';
                        keystatus[5] ='-';
                        staticPressure = pressure_avg;
                        break;
                    case 'z' :
                        // zero motion & position
                        keystatus[1] ='-';
                        keystatus[3] ='-';
                        keystatus[5] ='.';
                        strcpy(keystatus, " ----.----------------- \n");
                        motion->motionReset();
                        break;
                    case 'm' :
                        // magnetometer OFF
                        keystatus[7] ='m';
                        keystatus[9] ='m';
                        enableCompass = false;
                        imu->setCompassEnable(enableCompass);
                        break;
                    case 'M' :
                        // magnetometer ON
                        keystatus[7] ='M';
                        keystatus[9] ='M';
                        enableCompass = true;
                        imu->setCompassEnable(enableCompass);
                        break;
                    case 'g' :
                        // Gyro Bias runtime OFF
                        keystatus[9] ='g';
                        imu->setGyroRunTimeCalibrationEnable(false);
                        break;
                    case 'G' :
                        // runtime Gyro Bias ON
                        keystatus[9] ='G';
                        imu->setGyroRunTimeCalibrationEnable(true);
                        break;
                    case 'd' :
                        // Gyro Bias manual OFF
                        keystatus[9] ='d';
                        imu->setGyroManualCalibrationEnable(false);
                        break;
                    case 'D' :
                        // manual Gyro Bias ON
                        imu->setGyroRunTimeCalibrationEnable(false);
                        keystatus[9] ='D';
                        imu->setGyroManualCalibrationEnable(true);
                        break;
                    case 's' :
                        // trun streaming off
                        keystatus[9] ='s';
                        displayData = false;
                        break;
                    case 'S' :
                        // trun streaming on
                        keystatus[9] ='S';
                        displayData = true;
                        break;
                    case 'x' :
                        // must exit
                        keystatus[11] ='X';
                        mustExit = true;
                        break;
                    } // end switch input
            } // end if user pressed key
        } // user poll
    } // while
    printf("\nRTIMULibDrive11 exiting\n");
    settings->saveSettings(); // should update gyro bias
    return 0;
}

char getUserChar()
{
    int i;
    ioctl(0, FIONREAD, &i);
    if (i <= 0)
        return 0;
    // return tolower(getchar());
    return getchar();
}

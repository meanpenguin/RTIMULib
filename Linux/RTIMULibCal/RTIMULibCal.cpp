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


// RT Calibration Procedure
// read ini file into settings structure
// create IMU element
// set IMU Calibration Mode for ACCEL and COMPASS
// create mag calibration element
//  initialize this element (reset min/max, reset occtant counts)
// create accel calibration element
//  enable axis for calibration
//  if max min valid, set those to max min found, otherwise create empty data fro max min
//  initialize this element
// create temperature calibration element
//
//  depending on keyboard input
//   mag min/max calibration
//      reset min max
//      new minmax data
//      depending on keyboard ...
//  ...
//  Raw Data Mode is enabled through following mechanism:
//    in RTIMU there is Acceleration, Gyro and Compass Calibration and Bias Computations
//    the calibration routines check for valid calibration with
//         getAccelCalibrationValid() { return !m_accelCalibrationMode && m_settings->m_accelCalValid; }
//    if accelCalibrationMode or magCalibrationMode is true it will not apply max/min and Ellipsoid Calibration
//    When recording ellipsoid data, MagCal and AccelCal will conduct the max/min calibration the same was as 
//    RTIMU without using RTIMU routines. max min for accel and offset/scale for mag.
//    Setting CalibrationMode will disable application of calibration
//    However it will conduct axis rotations as defined for x/y/z for the individual IMUs
//

#include "RTIMULib.h"
#include "RTIMUMagCal.h"
#include "RTIMUAccelCal.h"
#include "RTIMUTemperatureCal.h"

#include <termios.h>
#include <unistd.h>
#include <ctype.h>
#include <sys/wait.h>
#include <sys/ioctl.h>

//  where to find the ellipsoid fitting code

#define ELLIPSOID_FIT_DIR               "../RTEllipsoidFit/"

//  function prototypes

void newIMU();
bool pollIMU();
char getUserChar();
void displayMenu();
void displayMagMinMax();
void displayMagEllipsoid();
void displayAccelMinMax();
void displayAccelEllipsoid();
void displayTemperature();

void doMagMinMaxCal();
void doMagEllipsoidCal();
void doAccelMinMaxCal();
void doAccelEllipsoidCal();
void doTemperatureCal();
void doRuntimeAccelCal();
void processEllipsoid();
void processAccelEllipsoid();
void processTemperature();

//  global variables

static RTIMUSettings *settings;
static RTIMU_DATA imuData;
static RTIMU *imu;
static RTIMUMagCal *magCal;
static RTIMUAccelCal *accelCal;
static RTIMUTemperatureCal *temperatureCal;
static bool magMinMaxDone;
static bool accelMinMaxDone;
static bool temperatureDone;

static bool accelEnables[3];
static bool accelEllipsoidEnable;
static int  accelCurrentAxis;

int main(int argc, char **argv)
{
    char *settingsFile;

    if (argc == 2)
        settingsFile = argv[1];
    else
        settingsFile = (char *)"RTIMULib";

    printf("RTIMULibCal - using %s.ini\n", settingsFile);

    settings = new RTIMUSettings(settingsFile);

    bool mustExit = false;
	
    imu = NULL;
    newIMU();

    //  set up for calibration run

    imu->setCompassCalibrationMode(true);
    imu->setAccelCalibrationMode(true);
    imu->setGyroCalibrationMode(true);
    imu->setTemperatureCalibrationMode(false);
    
    magCal = new RTIMUMagCal(settings);
    magCal->magCalInit();
    accelCal = new RTIMUAccelCal(settings);
    accelCal->accelCalInit();
    temperatureCal = new RTIMUTemperatureCal(settings);
    temperatureCal->temperatureCalInit();

    magMinMaxDone = false;
    accelMinMaxDone = false;
    temperatureDone = false;
    
    //  set up console io
    struct termios	ctty;
    tcgetattr(fileno(stdout), &ctty);
    ctty.c_lflag &= ~(ICANON);
    tcsetattr(fileno(stdout), TCSANOW, &ctty);

    //  the main loop
    while (!mustExit) {
        displayMenu();
        switch (getchar()) {
        case 'x' :
            mustExit = true;
            break;

        case 'm' :
            doMagMinMaxCal();
            break;

        case 'M' :
            doMagEllipsoidCal();
            break;

        case 'a' :
            doAccelMinMaxCal();
            break;

        case 'A' :
            doAccelEllipsoidCal();
            break;

        case 'T' :
            doTemperatureCal();
            break;

		case 'r' :
            doRuntimeAccelCal();
            break;
        }
    }

    printf("\nRTIMULibCal exiting\n");
    return 0;
}

void newIMU()
{
    if (imu != NULL)
        delete imu;

    imu = RTIMU::createIMU(settings);

    if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)) {
        printf("No IMU found\n");
        exit(1);
    }

    //  set up IMU

    imu->IMUInit();
}

void doTemperatureCal()
{
    uint64_t now;
    char input;

    /*
    if ( (magMinMaxDone == true) || (accelMinMaxDone == true)) {
        printf("\nYou cannot collect temperature data once min/max was done\n");
        printf("You might need to change CompassCalValid or AccelCalValid in the ini file.\n");
        printf("Compass: %d Accel %d\n", magMinMaxDone, accelMinMaxDone);
        return;
    }
    */
    
    temperatureCal->temperatureCalInit();
    imu->setTemperatureCalibrationMode(true);
    temperatureDone = false;

    //  now collect data

    printf("\nTemperature calibration\n");
    printf("-----------------------\n");
    printf("Blow hot and cold air onto the IMU chip.\n");
    printf("Keep temperature within reasonable limits.\n");
    printf("When enough data is recorded, enter 's' to save, 'r' to reset\n");
    printf("or 'x' to abort and discard the data.\n");
    printf("\nPress any key to start...");
    getchar();

    now = RTMath::currentUSecsSinceEpoch();

    while (1) {

        while (pollIMU()) {
            now = RTMath::currentUSecsSinceEpoch();
            if ( temperatureCal->newData(imuData.accel,imuData.gyro,imuData.compass,imuData.IMUtemperature) )
               displayTemperature();
        }

        if ((input = getUserChar()) != 0) {
            switch (input) {
            case 's' :
                printf("\nSaving temperature data.\n");
                
                if (temperatureCal->temperatureCalValid()) {
                    temperatureCal->temperatureCalSaveRaw(ELLIPSOID_FIT_DIR);
                    processTemperature();
                    temperatureDone = true;
                    imu->setTemperatureCalibrationMode(false);
                }
                return;

            case 'x' :
                printf("\nAborting.\n");
                return;

            case 'r' :
                printf("\nResetting temperature data.\n");
                temperatureCal->temperatureCalReset(); 
                imu->setTemperatureCalibrationMode(true);
                break;
            }
        }
        
        //  poll at the rate recommended by the IMU
        uint64_t time_elapsed = RTMath::currentUSecsSinceEpoch() - now;
        if ( (int)time_elapsed < (imu->IMUGetPollInterval()*1000) ) {
            usleep(imu->IMUGetPollInterval() * 1000 - time_elapsed);
        }
    }
}

void processTemperature()
{
    pid_t pid;
    int status;

    printf("\nProcessing temperature fit data ...\n");

    pid = fork();
    if (pid == 0) {
        //  child process
        chdir(ELLIPSOID_FIT_DIR);
        execl("/bin/sh", "/bin/sh", "-c", RTIMUCALDEFS_OCTAVE_COMMAND_TEMPERATURE, NULL);
        printf("here");
        _exit(EXIT_FAILURE);
    } else if (pid < 0) {
        printf("\nFailed to start temperature fitting code.\n");
        return;
    } else {
        //  parent process - wait for child
        if (waitpid(pid, &status, 0) != pid) {
            printf("\nTemperature fit failed, %d\n", status);
        } else {
            if (status == 0) {
                printf("\nTemperature fit completed - saving data to file.");
                if (temperatureCal->temperatureCalSaveCorr(ELLIPSOID_FIT_DIR) == false) {
                    printf("\nUnable to save temperature coefficients.\n");}
                else {
                    printf("\nTemperature coefficients saved.\n");
                };
            } else {
                printf("\nTemperature fit returned %d - aborting.\n", status);
            }
        }
    }
}

void doMagMinMaxCal()
{
    uint64_t displayTimer;
    uint64_t now;
    char input;

    magCal->magCalInit();
    magMinMaxDone = false;

    //  now collect data

    printf("\nMagnetometer min/max calibration\n");
    printf("--------------------------------\n");
    printf("Waggle the IMU chip around, ensuring that all six axes\n");
    printf("(+x, -x, +y, -y and +z, -z) go through their extrema.\n");
    printf("When all extrema have been achieved, enter 's' to save, 'r' to reset\n");
    printf("or 'x' to abort and discard the data.\n");
    printf("\nPress any key to start...");
    getchar();

    now = RTMath::currentUSecsSinceEpoch();
    displayTimer = now;

    while (1) {

        while (pollIMU()) {
		
            now = RTMath::currentUSecsSinceEpoch();
			
            magCal->newMinMaxData(imuData.compass);

            //  display 10 times per second

            if ((now - displayTimer) > 100000) {
                displayMagMinMax();
                displayTimer = now;
            }
        }

        if ((input = getUserChar()) != 0) {
            switch (input) {
            case 's' :
                printf("\nSaving min/max data.\n");
                magCal->magCalSaveMinMax();
                magMinMaxDone = true;
                return;

            case 'x' :
                printf("\nAborting.\n");
                return;

            case 'r' :
                printf("\nResetting min/max data.\n");
                magCal->magCalReset();
                break;
            }
        }
        
        //  poll at the rate recommended by the IMU
        uint64_t time_elapsed = RTMath::currentUSecsSinceEpoch() - now;
        if ( (int)time_elapsed < imu->IMUGetPollInterval()*1000) {
            usleep(imu->IMUGetPollInterval() * 1000 - time_elapsed);
        }
    }
}


void doMagEllipsoidCal()
{
    uint64_t now;
    char input;

    if (!magMinMaxDone) {
        printf("\nYou cannot collect ellipsoid data until magnetometer min/max\n");
        printf("calibration has been performed.\n");
        return;
    }

    printf("\nMagnetometer ellipsoid calibration\n");
    printf("\n----------------------------------\n");
    printf("Move the magnetometer around in as many poses as possible.\n");
    printf("The counts for each of the 8 pose quadrants will be displayed.\n");
    printf("When enough data (%d samples per octant) has been collected,\n", RTIMUCALDEFS_OCTANT_MIN_SAMPLES);
    printf("ellipsoid processing will begin.\n");
    printf("Enter 'x' at any time to abort and discard the data.\n");
    printf("\nPress any key to start...");
    getchar();

    now = RTMath::currentUSecsSinceEpoch();

    while (1) {

        while (pollIMU()) {
		
            now = RTMath::currentUSecsSinceEpoch();
			
            if (magCal->newEllipsoidData(imuData.compass) == true) {
                displayMagEllipsoid();
            }

            if (magCal->magCalEllipsoidValid()) {
                magCal->magCalSaveRaw(ELLIPSOID_FIT_DIR);
                processEllipsoid();
                return;
            }
        }

        if ((input = getUserChar()) != 0) {
            switch (input) {
            case 'x' :
                printf("\nAborting.\n");
                return;
            }
        }
        
        //  poll at the rate recommended by the IMU
        uint64_t time_elapsed = RTMath::currentUSecsSinceEpoch() - now;
        if ( (int)time_elapsed < imu->IMUGetPollInterval()*1000) {
            usleep(imu->IMUGetPollInterval() * 1000 - time_elapsed);
        }

    }
}

void processEllipsoid()
{
    pid_t pid;
    int status;

    printf("\nProcessing ellipsoid fit data for magnetometer ...\n");

    pid = fork();
    if (pid == 0) {
        //  child process
        chdir(ELLIPSOID_FIT_DIR);
        execl("/bin/sh", "/bin/sh", "-c", RTIMUCALDEFS_OCTAVE_COMMAND, NULL);
        printf("here");
        _exit(EXIT_FAILURE);
    } else if (pid < 0) {
        printf("\nFailed to start ellipsoid fitting code.\n");
        return;
    } else {
        //  parent process - wait for child
        if (waitpid(pid, &status, 0) != pid) {
            printf("\nEllipsoid fit failed, %d\n", status);
        } else {
            if (status == 0) {
                printf("\nEllipsoid fit completed - saving data to file.");
                magCal->magCalSaveCorr(ELLIPSOID_FIT_DIR);
            } else {
                printf("\nEllipsoid fit returned %d - aborting.\n", status);
            }
        }
    }
}

void doRuntimeAccelCal()
{
    uint64_t displayTimer;
    uint64_t now;
    char input;
	bool engageRuntimeCalib = false;

    printf("\nAccelerometer Runtime Calibration\n");
    printf("-----------------------------------\n");
    printf("Available options are:\n");
    printf("  R - enable runtime calibration.\n");
    printf("  r - disbale runtime calibration.\n");
    printf("  s - save the data once all 6 extrema have been collected.\n");
    printf("  x - abort and discard the data.\n");
    printf("\nPress any key to start...");
    getchar();
    
    now = RTMath::currentUSecsSinceEpoch();
    displayTimer = now;

    while (1) {

        while (pollIMU()) {

            now = RTMath::currentUSecsSinceEpoch();
            
            //  display 10 times per second

            if ((now - displayTimer) > 100000) {
				printf("\n");
			    printf("Accel x: %6.2f  y: %6.2f  z: %6.2f  s: %6.2f\n", imuData.accel.x(), imuData.accel.y(), imuData.accel.z(), imuData.accel.length());
				printf("Min x: %6.2f  min y: %6.2f  min z: %6.2f\n", settings->m_accelCalMin.data(0),
					settings->m_accelCalMin.data(1), settings->m_accelCalMin.data(2));
				printf("Max x: %6.2f  max y: %6.2f  max z: %6.2f\n", settings->m_accelCalMax.data(0),
					settings->m_accelCalMax.data(1), settings->m_accelCalMax.data(2));
				fflush(stdout);
                displayTimer = now;
            }
			
			if (engageRuntimeCalib) {
				imu->runtimeAdjustAccelCal();
			}
        }

        if ((input = getUserChar()) != 0) {
            switch (input) {
	
			case 'R':
				engageRuntimeCalib = true;
				break;

			case 'r':
				engageRuntimeCalib = false;
				break;

			 case 's' :
			    settings->saveSettings();
				accelMinMaxDone = true;
				accelCal->m_accelMin = settings->m_accelCalMin;
				accelCal->m_accelMax = settings->m_accelCalMax;
				printf("\nAccelerometer calibration data saved.\n");
				return;
		
			case 'x' :
				printf("\nAborting.\n");
				return;
			}
        }
        
        //  poll at the rate recommended by the IMU
        uint64_t time_elapsed = RTMath::currentUSecsSinceEpoch() - now;
        if ( (int)time_elapsed < imu->IMUGetPollInterval()*1000) {
            usleep(imu->IMUGetPollInterval() * 1000 - time_elapsed);
        }
	}
}

void doAccelMinMaxCal()
{
    uint64_t displayTimer;
    uint64_t now;
    char input;

    printf("\nAccelerometer Calibration\n");
    printf("-------------------------\n");
    printf("The code normally ignores readings until an axis has been enabled.\n");
    printf("The idea is to orient the IMU near the current extrema (+x, -x, +y, -y, +z, -z)\n");
    printf("and then enable the axis, moving the IMU very gently around to find the\n");
    printf("extreme value. Now disable the axis again so that the IMU can be inverted.\n");
    printf("When the IMU has been inverted, enable the axis again and find the extreme\n");
    printf("point. Disable the axis again and press the space bar to move to the next\n");
    printf("axis and repeat. The software will display the current axis and enable state.\n");
    printf("Available options are:\n");
    printf("  e - enable the current axis.\n");
    printf("  d - disable the current axis.\n");
    printf("  space bar - move to the next axis (x then y then z then x etc.\n");
    printf("  r - reset the current axis (if enabled).\n");
    printf("  s - save the data once all 6 extrema have been collected.\n");
    printf("  x - abort and discard the data.\n");
    printf("\nPress any key to start...");
    getchar();

    //  perform all axis reset
    for (int i = 0; i < 3; i++)
        accelCal->accelCalEnable(i, true);
    accelCal->accelCalReset();
    for (int i = 0; i < 3; i++)
        accelCal->accelCalEnable(i, false);

    accelCurrentAxis = 0;

    for (int i = 0; i < 3; i++)
        accelEnables[i] = false;

    now = RTMath::currentUSecsSinceEpoch();
    displayTimer = now;

    while (1) {

        while (pollIMU()) {

            now = RTMath::currentUSecsSinceEpoch();
            
            for (int i = 0; i < 3; i++)
                accelCal->accelCalEnable(i, accelEnables[i]);
            
            accelCal->newMinMaxData(imuData.accel);

            //  display 10 times per second

            if ((now - displayTimer) > 100000) {
                displayAccelMinMax();
                displayTimer = now;
            }
        }

        if ((input = getUserChar()) != 0) {
            switch (input) {
            case 'e' :
                accelEnables[accelCurrentAxis] = true;
                break;

            case 'd' :
                accelEnables[accelCurrentAxis] = false;
                break;

            case 'r' :
                accelCal->accelCalReset();
                break;

            case ' ' :
                if (++accelCurrentAxis == 3)
                    accelCurrentAxis = 0;
                break;

            case 's' :
                accelCal->accelCalSaveMinMax();
                accelMinMaxDone = true;
                printf("\nAccelerometer calibration data saved to file.\n");
                return;
    
            case 'x' :
                printf("\nAborting.\n");
                return;
            }
        }
        
        //  poll at the rate recommended by the IMU
        uint64_t time_elapsed = RTMath::currentUSecsSinceEpoch() - now;
        if ( (int)time_elapsed < imu->IMUGetPollInterval()*1000) {
            usleep(imu->IMUGetPollInterval() * 1000 - time_elapsed);
        }
    }
}

void doAccelEllipsoidCal()
{ 
    uint64_t now;
    char input;

    if (!accelMinMaxDone) {
        printf("\nYou cannot collect ellipsoid data until accelerometer min/max\n");
        printf("calibration has been performed.\n");
        return;
    }

    printf("\nAccelerometer ellipsoid calibration\n");
    printf("\n----------------------------------\n");
    printf("Move the accelerometer carefully around.\n");
    printf("Any rapid movements will falsely increase accelerometer readings.\n");
    printf("Please refer to supplementary instructions.\n");
    printf("The counts for each of the 8 pose quadrants will be displayed.\n");
    printf("When enough data (%d samples per octant) has been collected,\n", RTIMUCALDEFS_OCTANT_MIN_SAMPLES);
    printf("ellipsoid processing will begin.\n");
    printf("Enter 'x' at any time to abort and discard the data.\n");
    printf("Enter 'e' to enable data acquisition.\n");
    printf("Enter 'd' to disable/pause data acquisition.\n");
    printf("\nPress any key to start...");
    getchar();

    now = RTMath::currentUSecsSinceEpoch();

    accelEllipsoidEnable = true;
    
    while (1) {

        while (pollIMU()) {

            now = RTMath::currentUSecsSinceEpoch();

            if (accelEllipsoidEnable == true) {
                if (accelCal->newEllipsoidData(imuData.accel) == true) {
                    displayAccelEllipsoid();
                }
            }
            
            if (accelCal->accelCalEllipsoidValid()) {
                accelCal->accelCalSaveRaw(ELLIPSOID_FIT_DIR);
                processAccelEllipsoid(); 
                return;
            }
        }

        if ((input = getUserChar()) != 0) {
            switch (input) {
            case 'x' :
                printf("\nAborting.\n");
                return;
                
            case 'e' :
                accelEllipsoidEnable = true;
                break;

            case 'd' :
                accelEllipsoidEnable = false;
                break;

            }
        }

        //  poll at the rate recommended by the IMU
        uint64_t time_elapsed = RTMath::currentUSecsSinceEpoch() - now;
        if ( (int)time_elapsed < imu->IMUGetPollInterval()*1000) {
            usleep(imu->IMUGetPollInterval() * 1000 - time_elapsed);
        }

    }
}

void processAccelEllipsoid()
{
    pid_t pid;
    int status;

    printf("\nProcessing ellipsoid fit data for accelerometer ...\n");

    pid = fork();
    if (pid == 0) {
        //  child process
        chdir(ELLIPSOID_FIT_DIR);
        execl("/bin/sh", "/bin/sh", "-c", RTIMUCALDEFS_OCTAVE_COMMAND_ACCEL, NULL);
        printf("here");
        _exit(EXIT_FAILURE);
    } else if (pid < 0) {
        printf("\nFailed to start ellipsoid fitting code.\n");
        return;
    } else {
        //  parent process - wait for child
        if (waitpid(pid, &status, 0) != pid) {
            printf("\nEllipsoid fit failed, %d\n", status);
        } else {
            if (status == 0) {
                printf("\nEllipsoid fit completed - saving data to file.");
                accelCal->accelCalSaveCorr(ELLIPSOID_FIT_DIR);
            } else {
                printf("\nEllipsoid fit returned %d - aborting.\n", status);
            }
        }
    }
}

bool pollIMU()
{
    if (imu->IMURead()) {
        imuData = imu->getIMUData();
        return true;
    } else {
        return false;
    }
}

char getUserChar()
{
    int i;

    ioctl(0, FIONREAD, &i);
    if (i <= 0)
        return 0;
    return tolower(getchar());
}

void displayMenu()
{
    printf("\n");
    printf("Options are: \n");
    printf("  T - temperature calibration\n");
    printf("  m - calibrate magnetometer with min/max\n");
    printf("  M - calibrate magnetometer with ellipsoid (do min/max first)\n");
    printf("  a - calibrate accelerometers with min/max\n");
    printf("  A - calibrate accelerometers with ellipsoid (do min/max first)\n");
    printf("  r - runtime accelerometer calibration)\n");
    printf("  x - exit\n");
    printf("Enter option: ");
}

void displayTemperature()
{
    int index = temperatureCal->m_temperatureCalInIndex - 1;
    if (index < 0) { index=0; }

    TEMPERATURE_CAL_DATA data = temperatureCal->m_temperatureCalSamples[index];
    
    printf("\n");
    printf("IMU Temperature: %6.2f Max: %6.2f Min: %6.2f \n",  data.temperature, temperatureCal->m_temperatureMax, temperatureCal->m_temperatureMin);
    printf("Accel x: %6.2f  y: %6.2f  z: %6.2f\n", data.accel.x(), data.accel.y(), data.accel.z());
    printf("Gyro x: %6.2f  y: %6.2f  z: %6.2f\n", data.gyro.x(), data.gyro.y(), data.gyro.z());
    printf("Mag x: %6.2f  y: %6.2f  z: %6.2f\n", data.mag.x(), data.mag.y(), data.mag.z());
    printf("Count: %d\n", temperatureCal->m_temperatureCalCount);
    fflush(stdout);
}

void displayMagMinMax()
{
    printf("\n");
    printf("Min x: %6.2f  min y: %6.2f  min z: %6.2f\n", magCal->m_magMin.data(0),
           magCal->m_magMin.data(1), magCal->m_magMin.data(2));
    printf("Max x: %6.2f  max y: %6.2f  max z: %6.2f\n", magCal->m_magMax.data(0),
           magCal->m_magMax.data(1), magCal->m_magMax.data(2));
    fflush(stdout);
}

void displayMagEllipsoid()
{
    int counts[RTIMUCALDEFS_OCTANT_COUNT];

    printf("\n");

    magCal->magCalOctantCounts(counts);

    printf("---: %d  +--: %d  -+-: %d  ++-: %d\n", counts[0], counts[1], counts[2], counts[3]);
    printf("--+: %d  +-+: %d  -++: %d  +++: %d\n", counts[4], counts[5], counts[6], counts[7]);
    fflush(stdout);
}

void displayAccelMinMax()
{
    printf("\n");

    printf("Current axis: ");
    if (accelCurrentAxis == 0) {
        printf("x - %s", accelEnables[0] ? "enabled" : "disabled");
    } else     if (accelCurrentAxis == 1) {
        printf("y - %s", accelEnables[1] ? "enabled" : "disabled");
    } else     if (accelCurrentAxis == 2) {
        printf("z - %s", accelEnables[2] ? "enabled" : "disabled");
    }

    printf("\nMin x: %6.2f  min y: %6.2f  min z: %6.2f\n", accelCal->m_accelMin.data(0),
           accelCal->m_accelMin.data(1), accelCal->m_accelMin.data(2));
    printf("Max x: %6.2f  max y: %6.2f  max z: %6.2f\n", accelCal->m_accelMax.data(0),
           accelCal->m_accelMax.data(1), accelCal->m_accelMax.data(2));
    fflush(stdout);
}

void displayAccelEllipsoid()
{
    int counts[RTIMUCALDEFS_OCTANT_COUNT];

    printf("\n");

    accelCal->accelCalOctantCounts(counts);

    printf("---: %d  +--: %d  -+-: %d  ++-: %d\n", counts[0], counts[1], counts[2], counts[3]);
    printf("--+: %d  +-+: %d  -++: %d  +++: %d\n", counts[4], counts[5], counts[6], counts[7]);
    fflush(stdout);
}

# RTIMULib for Linux

This directory contains the applications for embedded Linux systems such as the Raspberry Pi and Intel Edison. This description assumes that the Edison image was built using the meta-edison-rt layer, available on the GitHub repo.

The RTIMULibvrpm demo app, which shows how to integrate RTIMULib with vrpn, has its own build and run instructions in a README.md within the RTIMULibvrpn directory.

### Setting up the Raspberry Pi

#### Connecting the IMU

The easiest way to connect the IMU to the Raspberry Pi is to use something like the Adafruit Pi Plate (http://www.adafruit.com/products/801) as it makes it obvious where the I2C bus 1 pins are and where to pick up 3.3V. Basically, you need to connect the I2C SDA, I2C SCL, 3.3V and GND to the IMU breakout board you are using. Take care with these connections or else disaster may follow!

#### Enabling and Configuring the I2C Bus

In RasPI Jessie under Preference COnfiguration enable the I2C and SPI bus
It’s  useful to install the I2C tools:

	sudo apt-get install i2c-tools

Then:

	sudo i2cdetect -y 1

will detect any devices on /dev/i2c-1. If you have the MPU9150 wired up, you should see it at address 0x68. This is the default address expected by the demo programs. If it is at 0x69, the address expected by the demo programs will need to be changed (there's a settings file for doing things like that so it's easy to do).

Another thing worth doing is to change the I2C bus speed to 400KHz. Add the following line to /boot/config.txt:

	dtparam=i2c_arm_baudrate=400000

Simplest thing is then to reboot to make this change.

The I2C bus should now be ready for operation.

The Raspberry Pi requires that cmake is installed. Enter:

    sudo apt-get install cmake

### Setting up the Intel Edison

It's necessary to add an extra directory to ldconfig. Create /etc/ld.so.conf and add the line:

    /usr/local/lib
    
Then run ldconfig so that the RTIMULib libraries will be found.

### Build using cmake

Go to the directory where RTIMULib was cloned and enter:

    mkdir build
    cd build
    cmake ..
    make -j4
    sudo make install
    sudo ldconfig
    
This will build and install all of the libraries and demo programs. Note that the Intel Edison does not need the "sudo" since the default user is root. The "-j4" part indicates how many cores should be used and will considerably speed up the build on the Pi2 and Edison.

### Build using make

Makefiles are provided for RTIMULibCal, RTIMULibDrive, RTIMULibDrive10 and RTIMULibDrive11. To build, navigate to the directory for the app and enter:

    make -j4
    sudo make install

### Build using qmake (Raspberry Pi only)

.pro files are included for RTIMULibDemo and RTIMULibDemoGL. To build using qmake, navigate to the RTIMULibDemo or RTIMULibDemoGL directory and enter:

    qmake
    make -j4
    sudo make install
    
The .pro files can also be used with QtCreator if desired.

### Run the RTIMULibCal App

RTIMULibCal can either add calibration data to an existing RTIMULib.ini or else create a new one with the calibration data. RTIMULib.ini is used/created in the working directory.

If magnetometer ellipsoid fit isn't required, RTIMULibCal can be run anywhere. If ellipsoid fit is required, then the program assumes that the RTEllipsoidFit directory is at the same level as the working directory so that "../RTEllipsoidFit" refers to the directory holding the RTEllipsoidFit.m octave program. If not, ellipsoid fitting will fail. Note - ellipsoid fit is not supported on the Intel Edison.

The normal process is to run the magnetometer min/max option followed by the magnetometer ellipsoid fit option followed finally by the accelerometer min/max option. The program is self-documenting in that the instructions for every option will be displayed when the option is selected.

The resulting RTIMULib.ini can then be used by any other RTIMULib application.

Please consult https://github.com/mjs513/FreeIMU-Updates/wiki/images/Rotations.png for proper rotations that will cover all quadrants. An exam[le approach is shown here: https://youtu.be/sf_8jcDyO0Q


### Run the RTIMULibDrive11 Demo App

RTIMULibDrive is a simple command line program that shows how simple it is to use RTIMULib. RTIMULibDrive10 extends this to also support 10-dof IMUs with pressure/temperature sensors. RTIMULibDrive11 adds humidity sensor support to RTIMULibDrive10.

You should be able to run the program just by entering RTIMULibDrive(10/11). It will try to auto detect the connected IMU If all is well, you should see a line showing the sample rate and the current Euler angles. This is updated 10 times per second, regardless of the sensor sample rate. By default, the driver runs at 50 samples per second in most cases. So, you should see the sample rate indicating around 50 samples per second. The sample rate can be changed by editing the .ini file entry for the appropriate IMU type.

The displayed pose shows the roll, pitch and yaw seen by the IMU. Using an aircraft analogy, the roll axis points from the pilot towards the nose, the pitch axis points from the pilot along the right wing and the yaw axis points from the pilot down towards the ground. Right wing down is a positive roll, nose up is a positive pitch and clockwise rotation is a positive yaw.

Various parameters can be changed by editing the RTIMULib.ini file. These are described later.

Take a look at RTIMULibDrive11.cpp. Quite a few of the code lines are just to calculate rates and display outputs!

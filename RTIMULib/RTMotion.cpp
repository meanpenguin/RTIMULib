#define DEBUGVELOCITYBIAS false

#include "RTMotion.h"

RTMotion::RTMotion(RTIMUSettings *settings)
{
    m_settings = settings;
    m_accnorm_avg = new RunningAverage(ACCEL_AVG_HISTORY);
    m_accnorm_var = new RunningAverage(ACCEL_VAR_HISTORY);
    m_heading_X_avg = new RunningAverage(HEADING_AVG_HISTORY);
    m_heading_Y_avg = new RunningAverage(HEADING_AVG_HISTORY);
}

RTMotion::~RTMotion()
{

}

void RTMotion::motionInit()
{
    m_timestamp_previous = RTMath::currentUSecsSinceEpoch();;
    m_worldResiduals.zero();
    m_worldResiduals_previous.zero();
    m_worldResiduals.zero();
    m_worldResiduals_previous.zero();
    m_worldVelocity.zero();
    m_worldVelocity_drift.zero();
    m_worldVelocity_previous.zero();
    m_worldPosition.zero();
    m_motion = false;
    m_motion_previous = false;

    m_MotionData.worldAcceleration.zero();
    m_MotionData.worldVelocity.zero();
    m_MotionData.worldPosition.zero();
    m_MotionData.motion = false;
    
}

void RTMotion::motionReset()
{
    m_worldVelocity.zero();
    m_worldPosition.zero();
    m_worldVelocity_previous.zero();
    m_worldVelocity_drift.zero();
    m_motion = false;
    
    m_MotionData.worldVelocity.zero();
    m_MotionData.worldPosition.zero();
    m_MotionData.motion = false;
}

void RTMotion::motionResetPosition()
{
    m_worldPosition.zero();
    m_MotionData.worldPosition.zero();
}

RTFLOAT RTMotion::updateAverageHeading(RTFLOAT& heading) 
{
    // this needs two steps because of 0 - 360 jump at North 
    m_heading_X_avg->addValue(cos(heading));
    m_heading_Y_avg->addValue(sin(heading));
    return atan2(m_heading_Y_avg->getAverage(),m_heading_X_avg->getAverage());
}

bool RTMotion::detectMotion(RTVector3& acc, RTVector3& gyr) {
// Three Stage Motion Detection
// Original Code is from FreeIMU Processing example
// Some modifications and tuning
//
// 1. Strong Acceleration
// 2. Acceleration Variance
// 3. Gyro activity
// Any of the three triggers motion
//
// This code works well in detecting motion but has delay in going back to no motion
// This code is useful for computing velocity and position from acceleration
// It can be used to trigger accelration integration when motion startes
// and can reset velocity when transitioning from motion to no motion
  
  bool omegax, omegay, omegaz;
  bool accnorm_test, accnorm_var_test, omega_test;
  
  // ACCELEROMETER
  ////////////////
  // Test for strong accelerometer signal deviating from gravity (|acc| = 1) 
  float accnorm = acc.squareLength(); 
  if((accnorm >=0.85) && (accnorm <= 1.15)){ accnorm_test = false; } else { accnorm_test = true; }

  // Test for sudden accelerometer changes, similar to high pass filter of acceleration
  m_accnorm_avg->addValue(accnorm); // compute the average acceleration
  float accnormavg = m_accnorm_avg->getAverage(); // get acceleration average from filter
  m_accnorm_var->addValue(pow((accnorm-accnormavg),2.0)); // compute deviation from average
  float accnorm_varavg=m_accnorm_var->getAverage(); // computer average of the deviation
  // Motion if average deviation from average acceleration exceed threshold 
  // the threshold depends on the noise of the sensor 
  if( accnorm_varavg < 0.0005) { accnorm_var_test = false; } else { accnorm_var_test = true; }

  // GYRO
  ////////////////
  //   angular rate analysis 
  if ((abs(gyr.x()) >=0.02) ) { omegax = true; } else { omegax = false; }
  if ((abs(gyr.y()) >=0.02) ) { omegay = true; } else { omegay = false; }
  if ((abs(gyr.z()) >=0.02) ) { omegaz = true; } else { omegaz = false; }
  if (omegax || omegay || omegaz) { omega_test = true; } else { omega_test = false; }

  // Combine acceleration test, acceleration deviation test and gyro test
  ///////////////////////////////////////////////////////////////////////
  if (accnorm_test || omega_test || accnorm_var_test) { 
    return true; 
  } else { 
    return false;
  }
  
}

void RTMotion::updateVelocityPosition(RTVector3& residuals, RTQuaternion& q, float accScale, uint64_t& timestamp, bool& motion)
{
// Input:
//  Acceleration Residuals
//  Quaternion
//  Acceleration Scale, usually 9.81 m/s/s
//  Motion status
// Output:
//  Acceleration in world coordinate system
//  Velocity in world coordinate system
//  Position in world coordinate system

    float dt;
    bool motion_ended = false;
    RTVector3 residuals_cal;
    
    // Integration Time Step
    ////////////////////////
    dt = ((float)(timestamp - m_timestamp_previous)) * 0.000001f; // in seconds
    m_timestamp_previous = timestamp;

    // Check on Motion
    ///////////////////
    //  Did it start?
    if (m_motion_previous == false && motion == true) {
            m_motionStart_time = timestamp; }
    //  Did it end?
    if (m_motion_previous == true && motion == false) {
            m_dtmotion = ((float)(timestamp - m_motionStart_time))* 0.000001f; // time is in microseconds
            motion_ended = true;
    } else {
            motion_ended = false; 
    }
    // Keep track of previous status
    m_motion_previous = motion;

    if ( motion == true) { 
    // integrate acceleration and velocity only of motion occurs

        // operate in the world coordinate system and spatial units
        residuals_cal = residuals * accScale; // scale from g to real spatial units
        m_worldResiduals = RTMath::toWorld(residuals_cal, q); // rotate residuals to world coordinate system

        // integrate acceleration and add to velocity (uses trapezoidal integration technique
        m_worldVelocity = m_worldVelocity_previous + ((m_worldResiduals + m_worldResiduals_previous)*0.5f* dt );

        // Update Velocity Bias
        // When motion ends, velocity should be zero
        if ((motion_ended == true) && (m_dtmotion > 0.5f)) { // update velocity bias if we had at least half of second motion
            // m_worldVelocity_bias=m_worldVelocity/m_dtmotion;
            // could use some learning averaging here with previous values and giving long motions more weight
            // the velocity bias is a drift
            m_worldVelocity_drift = ( (m_worldVelocity_drift * (1.0f - velocityDriftLearningAlpha)) + ((m_worldVelocity / m_dtmotion) * velocityDriftLearningAlpha ) );
        }

         #if DEBUGVELOCITYBIAS
           RTMath::display("Bias: ", worldVelocity_bias);
           printf(" time: "); printf("%f",m_dtmotion); 
         #endif

        // Update Velocity
        m_worldVelocity = m_worldVelocity - ( m_worldVelocity_drift * dt );

        // integrate velocity and add to position
        m_worldPosition = m_worldPosition + ((m_worldVelocity + m_worldVelocity_previous)*0.5f)*dt;

        // keep history of previous values
        m_worldResiduals_previous = m_worldResiduals;
        m_worldVelocity_previous  = m_worldVelocity;

    } else {
        m_worldVelocity.zero();    // minimize error propagation
    }

    m_MotionData.worldPosition = m_worldPosition;
    m_MotionData.worldVelocity = m_worldVelocity;
    m_MotionData.worldAcceleration = m_worldResiduals;
    
}
		 
#ifndef _Motion_H
#define	_Motion_H

#include "RTIMULib.h"
#include "RTIMULibDefs.h"
#include "RunningAverage.h"

#define ACCEL_AVG_HISTORY     5                      // size of moving average filter
#define ACCEL_VAR_HISTORY     7                      // size of moving average filter
#define HEADING_AVG_HISTORY   5                      // size of moving average filter 

# define velocityDriftLearningAlpha 0.2f

class RTMotion
{
public:
    // Creates motion object
    // Creates global motion structure
    //    and creates averaging filters
    RTMotion(RTIMUSettings *settings);
    virtual ~RTMotion();

    // Initialize motion data 
    // Call after creating Motion object
    void motionInit();

    // Reset Velocity and Position to Zero
    void motionReset();

    // Reset Position to Zero
    void motionResetPosition();

    // Updates heading averaging filter and returns average heading
    RTFLOAT updateAverageHeading(RTFLOAT& heading);

    // Based on 3 measures decided if motion occurred:
    // Absolute acceleration - gravity
    // Acceleration deviation from moving average
    // ABsotulte gyro values
    bool detectMotion(RTVector3& acc, RTVector3& gyr);

    // Computes world coordinate based residuals, velocity and position
    // Computes velocity drift when motion comes to halt, as velocity should be zero at that time
    void updateVelocityPosition(RTVector3& residuals, RTQuaternion& q, float accScale, uint64_t& timestamp, bool& motion);

    RTIMUSettings *m_settings;
    MOTION_DATA m_MotionData;                                   // the data from the Motion Processor
    
    const MOTION_DATA& getMotionData() { return m_MotionData; }
	
protected:
    
    uint64_t  m_timestamp_previous;
    RTVector3 m_worldVelocity;
    RTVector3 m_worldVelocity_drift;
    RTVector3 m_worldVelocity_previous;
    RTVector3 m_worldPosition;
    RTVector3 m_worldResiduals;
    RTVector3 m_worldResiduals_previous;
    bool      m_motion;
    bool      m_motion_previous;
    uint64_t  m_motionStart_time;
    float     m_dtmotion;
	
    static RunningAverage *m_accnorm_avg; // Running average for acceleration (motion detection)
    static RunningAverage *m_accnorm_var; // Running average for acceleration variance (motion detection)
    static RunningAverage *m_heading_X_avg; // Running average for heading (noise reduction)
    static RunningAverage *m_heading_Y_avg; // Running average for heading (noise reduction)
  
};
#endif // _Motion_H
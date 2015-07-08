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


// UU: This code was changed to
// Include accelerometer Ellipsoid compensation
// Ellipsoid routines are not yet tested

#include "RTIMUAccelCal.h"

//  ACCEL_ALPHA control the smoothing - the lower it is, the smoother it is

#define ACCEL_ALPHA                     0.1f

RTIMUAccelCal::RTIMUAccelCal(RTIMUSettings *settings)
{
    m_settings = settings;
    for (int i = 0; i < 3; i++)
        m_accelCalEnable[i] = false; // disable X, Y and Z calibration
}

RTIMUAccelCal::~RTIMUAccelCal()
{

}

void RTIMUAccelCal::accelCalInit()
{
    if (m_settings->m_accelCalValid) {
        m_accelMin = m_settings->m_accelCalMin;
        m_accelMax = m_settings->m_accelCalMax;
    } else {
        m_accelMin = RTVector3(RTIMUCALDEFS_DEFAULT_MIN, RTIMUCALDEFS_DEFAULT_MIN, RTIMUCALDEFS_DEFAULT_MIN);
        m_accelMax = RTVector3(RTIMUCALDEFS_DEFAULT_MAX, RTIMUCALDEFS_DEFAULT_MAX, RTIMUCALDEFS_DEFAULT_MAX);
    }
}

void RTIMUAccelCal::accelCalReset()
{

    for (int i = 0; i < 3; i++) {
        if (m_accelCalEnable[i]) {
            m_accelMin.setData(i, RTIMUCALDEFS_DEFAULT_MIN);
            m_accelMax.setData(i, RTIMUCALDEFS_DEFAULT_MAX);
        }
    }
}

void RTIMUAccelCal::accelCalEnable(int axis, bool enable)
{
    m_accelCalEnable[axis] = enable;
}

void RTIMUAccelCal::newMinMaxData(const RTVector3& data)
{
    if (m_startCount > 0) {
        m_startCount--;
        return;
    }

    for (int i = 0; i < 3; i++) {
        if (m_accelCalEnable[i]) {
            m_averageValue.setData(i, (data.data(i) * ACCEL_ALPHA + m_averageValue.data(i) * (1.0 - ACCEL_ALPHA)));
            if (m_accelMin.data(i) > data.data(i)) {
                m_accelMin.setData(i, data.data(i));
            }

            if (m_accelMax.data(i) < data.data(i)) {
                m_accelMax.setData(i, data.data(i));
            }
        }
    }
}

bool RTIMUAccelCal::accelCalValid()
{
    bool valid = true;

    for (int i = 0; i < 3; i++) {
        if (m_accelMax.data(i) < m_accelMin.data(i))
            valid = false;
    }
    return valid;
}

bool RTIMUAccelCal::accelCalSaveMinMax()
{
    if (!accelCalValid())
        return false;

    m_settings->m_accelCalValid = true;
    m_settings->m_accelCalMin = m_accelMin;
    m_settings->m_accelCalMax = m_accelMax;
    m_settings->m_accelCalEllipsoidValid = false;
    m_settings->saveSettings();
    
    //  need to invalidate ellipsoid data in order to use new min/max data

    m_accelCalCount = 0;
    for (int i = 0; i < RTIMUCALDEFS_OCTANT_COUNT; i++)
        m_octantCounts[i] = 0;
    m_accelCalInIndex = m_accelCalOutIndex = 0;

    // and set up for min/max calibration

    setMinMaxCal();

    return true;
}

void RTIMUAccelCal::newEllipsoidData(const RTVector3& data)
{
    RTVector3 calData;

    //  do min/max calibration first

    for (int i = 0; i < 3; i++)
        calData.setData(i, (data.data(i) - m_minMaxOffset.data(i)) * m_minMaxScale.data(i));

    //  now see if it's already there - we want them all unique and slightly separate (using a fuzzy compare)

    for (int index = m_accelCalOutIndex, i = 0; i < m_accelCalCount; i++) {
        if ((abs(calData.x() - m_accelCalSamples[index].x()) < RTIMUCALDEFS_ELLIPSOID_MIN_SPACING) &&
            (abs(calData.y() - m_accelCalSamples[index].y()) < RTIMUCALDEFS_ELLIPSOID_MIN_SPACING) &&
            (abs(calData.z() - m_accelCalSamples[index].z()) < RTIMUCALDEFS_ELLIPSOID_MIN_SPACING)) {
                return;                                         // too close to another sample
        }
        if (++index == RTIMUCALDEFS_MAX_ACC_SAMPLES)
            index = 0;
    }


    m_octantCounts[findOctant(calData)]++;

    m_accelCalSamples[m_accelCalInIndex++] = calData;
    if (m_accelCalInIndex == RTIMUCALDEFS_MAX_ACC_SAMPLES)
        m_accelCalInIndex = 0;

    if (++m_accelCalCount == RTIMUCALDEFS_MAX_ACC_SAMPLES) {
        // buffer is full - pull oldest
        removeAccelCalData();
    }
}

bool RTIMUAccelCal::accCalEllipsoidValid()
{
    bool valid = true;

    for (int i = 0; i < RTIMUCALDEFS_OCTANT_COUNT; i++) {
        if (m_octantCounts[i] < RTIMUCALDEFS_OCTANT_MIN_SAMPLES)
            valid = false;
    }
    return valid;
}

RTVector3 RTIMUAccelCal::removeAccelCalData()
{
    RTVector3 ret;

    if (m_accelCalCount == 0)
        return ret;

    ret = m_accelCalSamples[m_accelCalOutIndex++];
    if (m_accelCalOutIndex == RTIMUCALDEFS_MAX_ACC_SAMPLES)
        m_accelCalOutIndex = 0;
    m_accelCalCount--;
    m_octantCounts[findOctant(ret)]--;
    return ret;
}

bool RTIMUAccelCal::accelCalSaveRaw(const char *ellipsoidFitPath)
{
    FILE *file;
    char *rawFile;

    if (ellipsoidFitPath != NULL) {
        // need to deal with ellipsoid fit processing
        rawFile = (char *)malloc(strlen(RTIMUCALDEFS_ACCEL_RAW_FILE) + strlen(ellipsoidFitPath) + 2);
        sprintf(rawFile, "%s/%s", ellipsoidFitPath, RTIMUCALDEFS_ACCEL_RAW_FILE);
        if ((file = fopen(rawFile, "w")) == NULL) {
            HAL_ERROR("Failed to open ellipsoid fit raw data file\n");
            return false;
        }
        while (m_accelCalCount > 0) {
            RTVector3 sample = removeAccelCalData();
            fprintf(file, "%f %f %f\n", sample.x(), sample.y(), sample.z());
        }
        fclose(file);
    }
    return true;
}

bool RTIMUAccelCal::accelCalSaveCorr(const char *ellipsoidFitPath)
{
    FILE *file;
    char *corrFile;
    float a[3];
    float b[9];

    if (ellipsoidFitPath != NULL) {
        corrFile = (char *)malloc(strlen(RTIMUCALDEFS_ACCEL_CORR_FILE) + strlen(ellipsoidFitPath) + 2);
        sprintf(corrFile, "%s/%s", ellipsoidFitPath, RTIMUCALDEFS_ACCEL_CORR_FILE);
        if ((file = fopen(corrFile, "r")) == NULL) {
            HAL_ERROR("Failed to open ellipsoid fit correction data file\n");
            return false;
        }
        if (fscanf(file, "%f %f %f %f %f %f %f %f %f %f %f %f",
            a + 0, a + 1, a + 2, b + 0, b + 1, b + 2, b + 3, b + 4, b + 5, b + 6, b + 7, b + 8) != 12) {
            HAL_ERROR("Ellipsoid correction file didn't have 12 floats\n");
            fclose(file);
            return false;
        }
        fclose(file);
        m_settings->m_accelCalEllipsoidValid = true;
        m_settings->m_accelCalEllipsoidOffset = RTVector3(a[0], a[1], a[2]);
        memcpy(m_settings->m_accelCalEllipsoidCorr, b, 9 * sizeof(float));
        m_settings->saveSettings();
        return true;
    }
    return false;
}


void RTIMUAccelCal::accelCalOctantCounts(int *counts)
{
    memcpy(counts, m_octantCounts, RTIMUCALDEFS_OCTANT_COUNT * sizeof(int));
}

int RTIMUAccelCal::findOctant(const RTVector3& data)
{
    int val = 0;

    if (data.x() >= 0)
        val = 1;
    if (data.y() >= 0)
        val |= 2;
    if (data.z() >= 0)
        val |= 4;

    return val;
}

void RTIMUAccelCal::setMinMaxCal()
{
    float maxDelta = -1;
    float delta;

    //  find biggest range

    for (int i = 0; i < 3; i++) {
        if ((m_accelMax.data(i) - m_accelMin.data(i)) > maxDelta)
            maxDelta = m_accelMax.data(i) - m_accelMin.data(i);
    }
    if (maxDelta < 0) {
        HAL_ERROR("Error in min/max calibration data\n");
        return;
    }
    maxDelta /= 2.0f;                                       // this is the max +/- range

    for (int i = 0; i < 3; i++) {
        delta = (m_accelMax.data(i) -m_accelMin.data(i)) / 2.0f;
        m_minMaxScale.setData(i, maxDelta / delta);            // makes everything the same range
        m_minMaxOffset.setData(i, (m_accelMax.data(i) + m_accelMin.data(i)) / 2.0f);
    }
}

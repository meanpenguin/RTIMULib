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


// UU: This code was create to
// provide temperature bias compensation

#include "RTIMUTemperatureCal.h"

RTIMUTemperatureCal::RTIMUTemperatureCal(RTIMUSettings *settings)
{
    m_settings = settings;
}

RTIMUTemperatureCal::~RTIMUTemperatureCal()
{
//
}

void RTIMUTemperatureCal::temperatureCalInit()
{
    /*
    if (m_settings->m_temperatureCalValid) {
        // set local data from current settings
    } else {
        // empty
    }
    */
    
    m_temperatureCalCount = 0;
    m_temperatureCalInIndex = 0;
    m_temperatureCalOutIndex = 0;
    m_temperatureMax = -99.0;
    m_temperatureMin =  99.0;

}

void RTIMUTemperatureCal::temperatureCalReset()
{
    // set local data to defaults
    m_temperatureCalCount = 0;
    m_temperatureCalInIndex = 0;
    m_temperatureCalOutIndex = 0;
    m_temperatureMax = -99.0;
    m_temperatureMin =  99.0;
}

bool RTIMUTemperatureCal::newData(const RTVector3& accel, const RTVector3& gyro, const RTVector3& mag, const RTFLOAT& temperature)
{
    // ACCELEROMETER DATA
    // GYRO DATA
    // COMPASS DATA
    // TEMPERATURE DATA
    
    //  now see if it's already there - we want them all unique and slightly separate (using a fuzzy compare)
    for (int index = m_temperatureCalOutIndex, i = 0; i < m_temperatureCalCount; i++) {
        if ((fabs(temperature - m_temperatureCalSamples[index].temperature) < (RTFLOAT) RTIMUCALDEFS_TEMPERATURE_MIN_SPACING)) {
                return false;                                         // too close to another sample
        }
        if (++index == RTIMUCALDEFS_MAX_TEMPERATURE_SAMPLES)
            index = 0;
    }
    
    // the saved samples for temperature fit
    m_temperatureCalSamples[m_temperatureCalInIndex].temperature=temperature;
    m_temperatureCalSamples[m_temperatureCalInIndex].accel=accel;
    m_temperatureCalSamples[m_temperatureCalInIndex].gyro=gyro;
    m_temperatureCalSamples[m_temperatureCalInIndex].mag=mag;
    m_temperatureCalInIndex++;

    if (temperature > m_temperatureMax)
        m_temperatureMax = temperature;
    if (temperature < m_temperatureMin)
        m_temperatureMin = temperature;
    
    if (m_temperatureCalInIndex == RTIMUCALDEFS_MAX_TEMPERATURE_SAMPLES)
        m_temperatureCalInIndex = 0;

    if (++m_temperatureCalCount == RTIMUCALDEFS_MAX_TEMPERATURE_SAMPLES) {
        // buffer is full - pull oldest
        removeTemperatureCalData();
    }
    return true;
}

bool RTIMUTemperatureCal::temperatureCalValid()
{
    for (int index = m_temperatureCalOutIndex, i = 0; i < m_temperatureCalCount; i++) {
        if ( (m_temperatureCalSamples[index].temperature > 90.0f) || (m_temperatureCalSamples[index].temperature < -90.0f) ) {
                return false;                                         // too close to another sample
        }
        if (++index == RTIMUCALDEFS_MAX_TEMPERATURE_SAMPLES)
            index = 0;
    }
    return true;
}

TEMPERATURE_CAL_DATA RTIMUTemperatureCal::removeTemperatureCalData()
{
    TEMPERATURE_CAL_DATA ret;

    if (m_temperatureCalCount == 0)
        return ret;

    ret = m_temperatureCalSamples[m_temperatureCalOutIndex++];
    if (m_temperatureCalOutIndex == RTIMUCALDEFS_MAX_TEMPERATURE_SAMPLES)
        m_temperatureCalOutIndex = 0;
    m_temperatureCalCount--;
    return ret;
}

bool RTIMUTemperatureCal::temperatureCalSaveRaw(const char *ellipsoidFitPath)
{
    FILE *file;
    char *rawFile;

    if (ellipsoidFitPath != NULL) {
        // need to deal with ellipsoid fit processing
        rawFile = (char *)malloc(strlen(RTIMUCALDEFS_TEMPERATURE_RAW_FILE) + strlen(ellipsoidFitPath) + 2);
        sprintf(rawFile, "%s/%s", ellipsoidFitPath, RTIMUCALDEFS_TEMPERATURE_RAW_FILE);
        if ((file = fopen(rawFile, "w")) == NULL) {
            HAL_ERROR("Failed to open ellipsoid fit raw data file\n");
            return false;
        }
        while (m_temperatureCalCount > 0) {
            TEMPERATURE_CAL_DATA sample = removeTemperatureCalData();
            fprintf(file, "%f %f %f ", sample.accel.x(), sample.accel.y(), sample.accel.z());
            fprintf(file, "%f %f %f ", sample.gyro.x(), sample.gyro.y(), sample.gyro.z());
            fprintf(file, "%f %f %f ", sample.mag.x(), sample.mag.y(), sample.mag.z());
            fprintf(file, "%f\n", sample.temperature);
        }
        fclose(file);
    }
    return true;
}

bool RTIMUTemperatureCal::temperatureCalSaveCorr(const char *ellipsoidFitPath)
{
    FILE *file;
    char *corrFile;
    float c_mat[4][9];

    if (ellipsoidFitPath != NULL) {
        corrFile = (char *)malloc(strlen(RTIMUCALDEFS_TEMPERATURE_CORR_FILE) + strlen(ellipsoidFitPath) + 2);
        sprintf(corrFile, "%s/%s", ellipsoidFitPath, RTIMUCALDEFS_TEMPERATURE_CORR_FILE);
        if ((file = fopen(corrFile, "r")) == NULL) {
            HAL_ERROR("Failed to open temperature fit correction data file\n");
            return false;
        }
        if (fscanf(file, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f ",
                &c_mat[0][0], &c_mat[0][1], &c_mat[0][2], &c_mat[0][3], &c_mat[0][4], &c_mat[0][5], &c_mat[0][6], &c_mat[0][7], &c_mat[0][8],
                &c_mat[1][0], &c_mat[1][1], &c_mat[1][2], &c_mat[1][3], &c_mat[1][4], &c_mat[1][5], &c_mat[1][6], &c_mat[1][7], &c_mat[1][8],
                &c_mat[2][0], &c_mat[2][1], &c_mat[2][2], &c_mat[2][3], &c_mat[2][4], &c_mat[2][5], &c_mat[2][6], &c_mat[2][7], &c_mat[2][8],
                &c_mat[3][0], &c_mat[3][1], &c_mat[3][2], &c_mat[3][3], &c_mat[3][4], &c_mat[3][5], &c_mat[3][6], &c_mat[3][7], &c_mat[3][8] ) != 4*9) {
            HAL_ERROR("Temperature correction file didn't have 4*9 floats\n");
            fclose(file);
            return false;
        }
        fclose(file);
       
        m_settings->m_temperatureCalValid = true;
        for (int i = 0; i < 9; i++) {
            m_settings->m_c3[i] = c_mat[3][i];
            m_settings->m_c2[i] = c_mat[2][i];
            m_settings->m_c1[i] = c_mat[1][i];
            m_settings->m_c0[i] = c_mat[0][i];
        }
        m_settings->saveSettings();
        return true;
    }
    return false;
}

////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech
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

#ifndef _RTPRESSUREMS5803_H_
#define _RTPRESSUREMS5803_H_

#include "RTPressure.h"

//  State definitions

#define MS5803_STATE_IDLE               0
#define MS5803_STATE_TEMPERATURE        1
#define MS5803_STATE_PRESSURE           2
#define MS5803_STATE_RESET              3


class RTIMUSettings;

class RTPressureMS5803 : public RTPressure
{
public:
    RTPressureMS5803(RTIMUSettings *settings);
    ~RTPressureMS5803();

    virtual bool reset();
    virtual const char *pressureName() { return "MS5803"; }
    virtual int pressureType() { return RTPRESSURE_TYPE_MS5803; }
    virtual bool pressureInit();
    virtual bool pressureRead();
    virtual int  pressureGetPollInterval();

private:

    unsigned char m_pressureAddr;                           // I2C address

    int m_state;

    uint16_t m_calData[8];                                  // calibration data

    uint32_t m_D1;
    uint32_t m_D2;

    uint64_t m_timer;                                       // used to time conversions

};

#endif // _RTPRESSUREMS5803_H_


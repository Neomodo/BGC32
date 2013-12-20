/*

BGC32 from FocusFlight, a new alternative firmware
for the EvvGC controller

Original work Copyright (c) 2013 John Ihlein
                                 Alan K. Adamson

This file is part of BGC32.

Includes code and/or ideas from:

  1)BaseFlight
  2)EvvGC
  2)S.O.H. Madgwick

BGC32 is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

BGC32 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with EvvGC. If not, see <http://www.gnu.org/licenses/>.

*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

float accAngleSmooth[3];

uint8_t evvgcCFinitialized = false;

///////////////////////////////////////////////////////////////////////////////

void initOrientation()
{
    int initLoops = 150;
    float accAngle[NUMAXIS] = { 0.0f, 0.0f, 0.0f };
    int i;

    for (i = 0; i < initLoops; i++)
    {
        readMPU6050();

        computeMPU6050TCBias();

        sensors.accel500Hz[XAXIS] = ((float)rawAccel[XAXIS].value - accelTCBias[XAXIS]) * ACCEL_SCALE_FACTOR;
        sensors.accel500Hz[YAXIS] = ((float)rawAccel[YAXIS].value - accelTCBias[YAXIS]) * ACCEL_SCALE_FACTOR;
        sensors.accel500Hz[ZAXIS] = -((float)rawAccel[ZAXIS].value - accelTCBias[ZAXIS]) * ACCEL_SCALE_FACTOR;

        accAngle[ROLL]  += atan2f(-sensors.accel500Hz[YAXIS], -sensors.accel500Hz[ZAXIS]);
        accAngle[PITCH] += atan2f(sensors.accel500Hz[XAXIS], -sensors.accel500Hz[ZAXIS]);

        accAngleSmooth[ROLL ] = accAngle[ROLL ] / (float)initLoops;
        accAngleSmooth[PITCH] = accAngle[PITCH] / (float)initLoops;

        delay(2);
    }

    sensors.evvgcCFAttitude500Hz[PITCH] = accAngleSmooth[PITCH];
    sensors.evvgcCFAttitude500Hz[ROLL ] = accAngleSmooth[ROLL ];
    sensors.evvgcCFAttitude500Hz[YAW  ] = 0.0f;
}

///////////////////////////////////////////////////////////////////////////////

void getOrientation(float *smoothAcc, float *orient, float *accData, float *gyroData, float dt)
{
    float accAngle[3];
    float gyroRate[3];

    if (evvgcCFinitialized == false)
    {
        initOrientation();

        evvgcCFinitialized = true;
    }

    //-------------------------------------------

    if (evvgcCFinitialized == true)
    {
        accAngle[ROLL ] = atan2f(-accData[YAXIS], -accData[ZAXIS]);
        accAngle[PITCH] = atan2f(accData[XAXIS], -accData[ZAXIS]);

        smoothAcc[ROLL]  = ((smoothAcc[ROLL ] * 99.0f) + accAngle[ROLL ]) / 100.0f;
        smoothAcc[PITCH] = ((smoothAcc[PITCH] * 99.0f) + accAngle[PITCH]) / 100.0f;

        gyroRate[PITCH] =  gyroData[PITCH];
        orient[PITCH]   = (orient[PITCH] + gyroRate[PITCH] * dt) + 0.0002f * (smoothAcc[PITCH] - orient[PITCH]);

        gyroRate[ROLL]  =  gyroData[ROLL] * cosf(fabsf(orient[PITCH])) + gyroData[YAW] * sinf(orient[PITCH]);
        orient[ROLL]    = (orient[ROLL] + gyroRate[ROLL] * dt) + 0.0002f * (smoothAcc[ROLL] - orient[ROLL]);

        gyroRate[YAW]   =  gyroData[YAW] * cosf(fabsf(orient[PITCH])) - gyroData[ROLL] * sinf(orient[PITCH]);
        orient[YAW]     = (orient[YAW] + gyroRate[YAW] * dt);
    }
}

///////////////////////////////////////////////////////////////////////////////

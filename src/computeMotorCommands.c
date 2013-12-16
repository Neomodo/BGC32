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

float mechanical2electricalDegrees[3] = { 1.0f, 1.0f, 1.0f };
float electrical2mechanicalDegrees[3] = { 1.0f, 1.0f, 1.0f };

float outputRate[3];

float pidCmd[3];

float pidCmdPrev[3] = { 0.0f, 0.0f, 0.0f };

///////////////////////////////////////////////////////////////////////////////
// Compute Motor Commands
///////////////////////////////////////////////////////////////////////////////

void computeMotorCommands(float dt)
{
	holdIntegrators = false;

	///////////////////////////////////

    if (activeRollState == true)
    {
    	pidCmd[ROLL] = updatePID(pointingCmd[ROLL] * mechanical2electricalDegrees[ROLL],
    	               sensors.margAttitude500Hz[ROLL] * mechanical2electricalDegrees[ROLL],
    	               dt, holdIntegrators, &eepromConfig.PID[ROLL_PID]);

    	outputRate[ROLL] = pidCmd[ROLL] - pidCmdPrev[ROLL];

	    if (outputRate[ROLL] > eepromConfig.rateLimit)
	        pidCmd[ROLL] = pidCmdPrev[ROLL] + eepromConfig.rateLimit;

	    if (outputRate[ROLL] < -eepromConfig.rateLimit)
	        pidCmd[ROLL] = pidCmdPrev[ROLL] - eepromConfig.rateLimit;

	    pidCmdPrev[ROLL] = pidCmd[ROLL];

	    setRollMotor(pidCmd[ROLL], (int)eepromConfig.rollPower);
    }

    ///////////////////////////////////

    if (activePitchState == true)
    {
        pidCmd[PITCH] = updatePID(pointingCmd[PITCH] * mechanical2electricalDegrees[PITCH],
        		                  sensors.margAttitude500Hz[PITCH] * mechanical2electricalDegrees[PITCH],
                                  dt, holdIntegrators, &eepromConfig.PID[PITCH_PID]);

	    outputRate[PITCH] = pidCmd[PITCH] - pidCmdPrev[PITCH];

	    if (outputRate[PITCH] > eepromConfig.rateLimit)
	        pidCmd[PITCH] = pidCmdPrev[PITCH] + eepromConfig.rateLimit;

	    if (outputRate[PITCH] < -eepromConfig.rateLimit)
	        pidCmd[PITCH] = pidCmdPrev[PITCH] - eepromConfig.rateLimit;

	    pidCmdPrev[PITCH] = pidCmd[PITCH];

	    setPitchMotor(pidCmd[PITCH], (int)eepromConfig.pitchPower);
    }

    ///////////////////////////////////

    if (activeYawState == true)
    {
        pidCmd[YAW] = updatePID(pointingCmd[YAW] * mechanical2electricalDegrees[YAW],
        		                sensors.margAttitude500Hz[YAW] * mechanical2electricalDegrees[YAW],
                                dt, holdIntegrators, &eepromConfig.PID[YAW_PID]);

	    outputRate[YAW] = pidCmd[YAW] - pidCmdPrev[YAW];

	    if (outputRate[YAW] > eepromConfig.rateLimit)
	        pidCmd[YAW] = pidCmdPrev[YAW] + eepromConfig.rateLimit;

	    if (outputRate[YAW] < -eepromConfig.rateLimit)
	        pidCmd[YAW] = pidCmdPrev[YAW] - eepromConfig.rateLimit;

	    pidCmdPrev[YAW] = pidCmd[YAW];

        setYawMotor(pidCmd[YAW], (int)eepromConfig.yawPower);
    }

    ///////////////////////////////////

}

///////////////////////////////////////////////////////////////////////////////

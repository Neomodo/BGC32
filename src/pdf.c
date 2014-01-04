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

void initPDF(void)
{
    uint8_t index;

    for (index = 0; index < NUMBER_OF_PDFS; index++)
    {
    	eepromConfig.PDF[index].iTerm = 0.0f;
	}
}

///////////////////////////////////////////////////////////////////////////////

float updatePDF(float command, float state, float deltaT, uint8_t iHold, struct PDFdata *PDFparameters)
{
    float error;

    ///////////////////////////////////

    error = command - state;

    ///////////////////////////////////

    if (iHold == false)
    {
    	PDFparameters->iTerm += error * deltaT;
    	PDFparameters->iTerm = constrain(PDFparameters->iTerm, -PDFparameters->windupGuard, PDFparameters->windupGuard);
    }

    ///////////////////////////////////

    return(PDFparameters->I * PDFparameters->iTerm - PDFparameters->P * state);

    ///////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////

void setPDFintegralError(uint8_t IDPid, float value)
{
	eepromConfig.PID[IDPid].iTerm = value;
}

///////////////////////////////////////////////////////////////////////////////

void zeroPDFintegralError(void)
{
    uint8_t index;

    for (index = 0; index < NUMBER_OF_PIDS; index++)
         setPIDintegralError(index, 0.0f);
}

///////////////////////////////////////////////////////////////////////////////



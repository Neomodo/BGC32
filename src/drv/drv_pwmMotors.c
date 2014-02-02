/*
  Sept 2013

  bgc32 Rev -

  Copyright (c) 2013 John Ihlein.  All rights reserved.

  Open Source STM32 Based Brushless Gimbal Controller Software

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick
  6)UAVX

  Designed to run on the EvvGC Brushless Gimbal Controller Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

// TIM8 Roll
// PC6, PC7, PC8 used for Roll,  TIM_OCPolarity_High
// PA7, PB0, PB1 used for RollN, TIM_OCPolarity_High

#define ROLL_A_GPIO         GPIOC
#define ROLL_A_PIN          GPIO_Pin_6
#define ROLL_A_PIN_SOURCE   GPIO_PinSource6
#define ROLL_B_GPIO         GPIOC
#define ROLL_B_PIN          GPIO_Pin_7
#define ROLL_B_PIN_SOURCE   GPIO_PinSource7
#define ROLL_C_GPIO         GPIOC
#define ROLL_C_PIN          GPIO_Pin_8
#define ROLL_C_PIN_SOURCE   GPIO_PinSource8

#define ROLL_AN_GPIO        GPIOA
#define ROLL_AN_PIN         GPIO_Pin_7
#define ROLL_AN_PIN_SOURCE  GPIO_PinSource7
#define ROLL_BN_GPIO        GPIOB
#define ROLL_BN_PIN         GPIO_Pin_0
#define ROLL_BN_PIN_SOURCE  GPIO_PinSource0
#define ROLL_CN_GPIO        GPIOB
#define ROLL_CN_PIN         GPIO_Pin_1
#define ROLL_CN_PIN_SOURCE  GPIO_PinSource1

// TIM1 Pitch
// PA8,  PA9,  PA10 used for Pitch,  TIM_OCPolarity_High
// PB13, PB14, PB15 used for PitchN, TIM_OCPolarity_High

#define PITCH_A_GPIO         GPIOA
#define PITCH_A_PIN          GPIO_Pin_8
#define PITCH_A_PIN_SOURCE   GPIO_PinSource8
#define PITCH_B_GPIO         GPIOA
#define PITCH_B_PIN          GPIO_Pin_9
#define PITCH_B_PIN_SOURCE   GPIO_PinSource9
#define PITCH_C_GPIO         GPIOA
#define PITCH_C_PIN          GPIO_Pin_10
#define PITCH_C_PIN_SOURCE   GPIO_PinSource10

#define PITCH_AN_GPIO        GPIOB
#define PITCH_AN_PIN         GPIO_Pin_13
#define PITCH_AN_PIN_SOURCE  GPIO_PinSource13
#define PITCH_BN_GPIO        GPIOB
#define PITCH_BN_PIN         GPIO_Pin_14
#define PITCH_BN_PIN_SOURCE  GPIO_PinSource14
#define PITCH_CN_GPIO        GPIOB
#define PITCH_CN_PIN         GPIO_Pin_15
#define PITCH_CN_PIN_SOURCE  GPIO_PinSource15

// TIM5 Yaw
// PA0, PA1, PA2 used for Yaw,  TIM_OCPolarity_High

#define YAW_A_GPIO         GPIOA
#define YAW_A_PIN          GPIO_Pin_0
#define YAW_A_PIN_SOURCE   GPIO_PinSource0
#define YAW_B_GPIO         GPIOA
#define YAW_B_PIN          GPIO_Pin_1
#define YAW_B_PIN_SOURCE   GPIO_PinSource1
#define YAW_C_GPIO         GPIOA
#define YAW_C_PIN          GPIO_Pin_2
#define YAW_C_PIN_SOURCE   GPIO_PinSource2

// TIM4 YawN
// PB6, PB7, PB8 used for YawN, TIM_OCPolarity_Low

#define YAW_AN_GPIO        GPIOB
#define YAW_AN_PIN         GPIO_Pin_6
#define YAW_AN_PIN_SOURCE  GPIO_PinSource6
#define YAW_BN_GPIO        GPIOB
#define YAW_BN_PIN         GPIO_Pin_7
#define YAW_BN_PIN_SOURCE  GPIO_PinSource7
#define YAW_CN_GPIO        GPIOB
#define YAW_CN_PIN         GPIO_Pin_8
#define YAW_CN_PIN_SOURCE  GPIO_PinSource8

///////////////////////////////////////

#define PWM_PERIOD 1000

#define MAX_CNT (PWM_PERIOD * 8 / 10)

#define BB_PERIPH_ADDR(addr, bit) ((vu32*)(PERIPH_BB_BASE + ((void*)(addr)-(void*)PERIPH_BASE) * 32 + (bit) * 4))

///////////////////////////////////////

static uint8_t pwmMotorDriverInitDone = false;

///////////////////////////////////////

int deadTimeRegister  = 144;  //   82;  // TIM_ClockDivision = TIM_CKD_DIV2, Tdts = 1 / (36 MHz)
                                        // Tdtg = Tdts = 27.78 nSec
                                        // deadtime = 82 * Tdtg = 2.278 uSec

int deadTimeDelay     = 160;  //   82;  // 36 MHz ticks for center aligned PWM mode
                                        // deadtime = 82 / 36E6 = 2.278 uSec

int halfDeadTimeDelay =  80;  //   41;  // 36 MHz ticks for center aligned PWM mode
                                        // halfDeadtime = 41 / 36E6 =  1.139 uSec

///////////////////////////////////////

int rollPhase[3], pitchPhase[3], yawPhase[6];

///////////////////////////////////////////////////////////////////////////////
//  IRQ Setup
///////////////////////////////////////////////////////////////////////////////

static void setupPWMIrq(uint8_t irq)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel                   = irq;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;       //Preemption Priority
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
}

///////////////////////////////////////////////////////////////////////////////
//  TIM8 IRQ Handler (ROLL)
///////////////////////////////////////////////////////////////////////////////

void TIM8_UP_IRQHandler(void)
{
	if ((TIM8->SR & TIM_SR_UIF) && !(TIM8->CR1 & TIM_CR1_DIR))  // if UIF flag is set and counting up
	{
    	__disable_irq();

        if (eepromConfig.rollEnabled)
        {
            TIM8->CCR1 = rollPhase[0];
            TIM8->CCR2 = rollPhase[1];
            TIM8->CCR3 = rollPhase[2];
        }
        else
        {
            TIM8->CCR1 = 0;
            TIM8->CCR2 = 0;
            TIM8->CCR3 = 0;
        }

        TIM8->DIER &= ~TIM_DIER_UIE; // disable update interrupt

        __enable_irq();
    }

	TIM8->SR &= ~TIM_SR_UIF; // clear UIF flag
}

///////////////////////////////////////////////////////////////////////////////
//  TIM1 IRQ Handler (PITCH)
///////////////////////////////////////////////////////////////////////////////

void TIM1_UP_IRQHandler(void)
{
	if ((TIM1->SR & TIM_SR_UIF) && !(TIM1->CR1 & TIM_CR1_DIR))  // if UIF flag is set and counting up
    {
		__disable_irq();

        if (eepromConfig.pitchEnabled)
        {
            TIM1->CCR1 = pitchPhase[0];
            TIM1->CCR2 = pitchPhase[1];
            TIM1->CCR3 = pitchPhase[2];
        }
        else
        {
            TIM1->CCR1 = 0;
            TIM1->CCR2 = 0;
            TIM1->CCR3 = 0;
        }

        TIM1->DIER &= ~TIM_DIER_UIE; // disable update interrupt

        __enable_irq();
    }

	TIM1->SR &= ~TIM_SR_UIF; // clear UIF flag
}

///////////////////////////////////////////////////////////////////////////////
//  TIM5 IRQ Handler (YAW)
///////////////////////////////////////////////////////////////////////////////

void TIM5_IRQHandler(void)
{
    if ((TIM5->SR & TIM_SR_UIF) && !(TIM5->CR1 & TIM_CR1_DIR))  // if UIF flag is set and counting up
    {
        __disable_irq();

        if (eepromConfig.yawEnabled)
        {
            TIM5->CCR1 = yawPhase[0];  // A Phase
            TIM5->CCR2 = yawPhase[1];  // B Phase
            TIM5->CCR3 = yawPhase[2];  // C Phase

            TIM4->CCR1 = yawPhase[3];  // AN Phase
            TIM4->CCR2 = yawPhase[4];  // BN Phase
            TIM4->CCR3 = yawPhase[5];  // CN Phase
        }
        else
        {
            TIM5->CCR1 = 0;
            TIM5->CCR2 = 0;
            TIM5->CCR3 = 0;

            TIM4->CCR1 = 0;
            TIM4->CCR2 = 0;
            TIM4->CCR3 = 0;
        }

        TIM5->DIER &= ~TIM_DIER_UIE;  // disable update interrupt

        __enable_irq();
    }

    TIM5->SR &= ~TIM_SR_UIF; // clear UIF flag
}

///////////////////////////////////////////////////////////////////////////////
//  Timer Channel Configuration
///////////////////////////////////////////////////////////////////////////////

static void timerChannelConfig(TIM_TypeDef *tim, TIM_OCInitTypeDef* OCInitStructure)
{
    TIM_OC1Init(tim, OCInitStructure);
    TIM_OC2Init(tim, OCInitStructure);
    TIM_OC3Init(tim, OCInitStructure);

	TIM_OC1PreloadConfig(tim, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(tim, TIM_OCPreload_Enable);
}

///////////////////////////////////////////////////////////////////////////////
//  Advanced Timer Configuration
///////////////////////////////////////////////////////////////////////////////

static void timerPWMadvancedConfig(TIM_TypeDef *tim)
{
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef       	TIM_OCInitStructure;
    TIM_BDTRInitTypeDef 		TIM_BDTRInitStructure;

    //Time Base configuration
    TIM_TimeBaseInitStructure.TIM_Prescaler         = 1;                               // 36 MHz
    TIM_TimeBaseInitStructure.TIM_CounterMode       = TIM_CounterMode_CenterAligned1;
    TIM_TimeBaseInitStructure.TIM_Period            = PWM_PERIOD - 1;                  // 36 MHz / (2 * 1000) = 18 kHz
    TIM_TimeBaseInitStructure.TIM_ClockDivision     = TIM_CKD_DIV2;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(tim, &TIM_TimeBaseInitStructure);

    //Configuration in PWM mode
    TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse        = 0;
    TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

    timerChannelConfig(tim, &TIM_OCInitStructure);

	//Automatic Output enable, Break, dead time and lock configuration
	TIM_BDTRInitStructure.TIM_OSSRState       = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState       = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel       = TIM_LOCKLevel_1;
	TIM_BDTRInitStructure.TIM_DeadTime        = deadTimeRegister;
	TIM_BDTRInitStructure.TIM_Break           = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity   = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;

	TIM_BDTRConfig(tim, &TIM_BDTRInitStructure);
}

///////////////////////////////////////////////////////////////////////////////
//  General Timer Configuration
///////////////////////////////////////////////////////////////////////////////

static void timerPWMgeneralConfig(TIM_TypeDef *tim, int polarity)
{
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef       	TIM_OCInitStructure;

    TIM_TimeBaseInitStructure.TIM_Prescaler         = 1;                               // 36 MHz
    TIM_TimeBaseInitStructure.TIM_CounterMode       = TIM_CounterMode_CenterAligned1;
    TIM_TimeBaseInitStructure.TIM_Period            = PWM_PERIOD - 1;                  // 36 MHz / (2 * 1000) = 18 kHz
    TIM_TimeBaseInitStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(tim, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;
    TIM_OCInitStructure.TIM_OCPolarity  = polarity;

    timerChannelConfig(tim, &TIM_OCInitStructure);
}

///////////////////////////////////////////////////////////////////////////////
//  Set PWM Via Table Lookup
///////////////////////////////////////////////////////////////////////////////

void setPWMFastTable(int *pwm, float angle, float power, uint8_t reverse)
{
    if (testPhase >= 0)
    {
        angle = testPhase;
    }

    int angleInt = (int)round(angle / M_TWOPI * SINARRAYSIZE);

	angleInt = angleInt % SINARRAYSIZE;

	if (angleInt < 0)
	{
		angleInt = SINARRAYSIZE + angleInt;
	}

	int iPower = (int)((PWM_PERIOD / 2 - deadTimeDelay) * power / 100);

    if (reverse == true)
    {
    	pwm[0] = (PWM_PERIOD / 2) - ((sinDataI16[ angleInt                               % SINARRAYSIZE] * iPower + SINARRAYSCALE / 2) / SINARRAYSCALE);
        pwm[2] = (PWM_PERIOD / 2) - ((sinDataI16[(angleInt +  1 * SINARRAYSIZE / 3)      % SINARRAYSIZE] * iPower + SINARRAYSCALE / 2) / SINARRAYSCALE);
        pwm[1] = (PWM_PERIOD / 2) - ((sinDataI16[(angleInt + (2 * SINARRAYSIZE + 1) / 3) % SINARRAYSIZE] * iPower + SINARRAYSCALE / 2) / SINARRAYSCALE);
    }
    else
    {
        pwm[0] = (PWM_PERIOD / 2) - ((sinDataI16[ angleInt                               % SINARRAYSIZE] * iPower + SINARRAYSCALE / 2) / SINARRAYSCALE);
        pwm[1] = (PWM_PERIOD / 2) - ((sinDataI16[(angleInt +  1 * SINARRAYSIZE / 3)      % SINARRAYSIZE] * iPower + SINARRAYSCALE / 2) / SINARRAYSCALE);
        pwm[2] = (PWM_PERIOD / 2) - ((sinDataI16[(angleInt + (2 * SINARRAYSIZE + 1) / 3) % SINARRAYSIZE] * iPower + SINARRAYSCALE / 2) / SINARRAYSCALE);
    }
}

///////////////////////////////////////////////////////////////////////////////
//  Set PWM
///////////////////////////////////////////////////////////////////////////////

void setPWM(int *pwm, float angle, float power, uint8_t reverse)
{
	setPWMFastTable(pwm, angle, power, reverse);
}

///////////////////////////////////////////////////////////////////////////////
//  Set PWM Data
///////////////////////////////////////////////////////////////////////////////

void setPWMData(int *target, int *pwm)
{
    __disable_irq();

    target[0] = pwm[0];
    target[1] = pwm[1];
    target[2] = pwm[2];

    __enable_irq();
}

///////////////////////////////////////////////////////////////////////////////
//  Set Yaw PWM Data
///////////////////////////////////////////////////////////////////////////////

void setYawPWMData(int *target, int *pwm)
{
    __disable_irq();

    target[0] = pwm[0] + halfDeadTimeDelay;
    target[1] = pwm[1] + halfDeadTimeDelay;
    target[2] = pwm[2] + halfDeadTimeDelay;
    target[3] = pwm[0] - halfDeadTimeDelay;
    target[4] = pwm[1] - halfDeadTimeDelay;
    target[5] = pwm[2] - halfDeadTimeDelay;

    __enable_irq();
}

///////////////////////////////////////////////////////////////////////////////
//  Activate Interrupt
///////////////////////////////////////////////////////////////////////////////

void activateIRQ(TIM_TypeDef *tim)
{
	__disable_irq();
	tim->SR &= ~TIM_SR_UIF;   // clear UIF flag
	tim->DIER = TIM_DIER_UIE; // Enable update interrupt
	__enable_irq();
}

///////////////////////////////////////////////////////////////////////////////
//  Set Roll Axis PWM
///////////////////////////////////////////////////////////////////////////////

void setRollMotor(float phi, float power)
{
	int pwm[3];

	setPWM(pwm, phi, power, (uint8_t)eepromConfig.rollReverse);
	setPWMData(rollPhase, pwm);
	activateIRQ(TIM8);
}

///////////////////////////////////////////////////////////////////////////////
//  Set Pitch Axis PWM
///////////////////////////////////////////////////////////////////////////////

void setPitchMotor(float theta, float power)
{
	int pwm[3];

	setPWM(pwm, theta, power, (uint8_t)eepromConfig.pitchReverse);
	setPWMData(pitchPhase, pwm);
	activateIRQ(TIM1);
}

///////////////////////////////////////////////////////////////////////////////
//  Set Yaw Axis PWM
///////////////////////////////////////////////////////////////////////////////

void setYawMotor(float psi, float power)
{
	int pwm[3];
	setPWM(pwm, psi, power, (uint8_t)eepromConfig.yawReverse);
	setYawPWMData(yawPhase, pwm);
	activateIRQ(TIM5);
}

///////////////////////////////////////////////////////////////////////////////
//  Force Motor Update
///////////////////////////////////////////////////////////////////////////////

void forceMotorUpdate(void)
{
    activateIRQ(TIM8);
    activateIRQ(TIM1);
    activateIRQ(TIM5);
}

///////////////////////////////////////////////////////////////////////////////
//  Initialize PWM Motor Drivers
///////////////////////////////////////////////////////////////////////////////

void pwmMotorDriverInit(void)
{
    if (pwmMotorDriverInitDone)
    {
        forceMotorUpdate();
        // make sure this init function is not called twice
        return;
    }

    GPIO_InitTypeDef  GPIO_InitStructure;

    ///////////////////////////////////

    // Roll PWM Timer Initialization here

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

    GPIO_InitStructure.GPIO_Pin   = ROLL_A_PIN | ROLL_B_PIN | ROLL_C_PIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = ROLL_AN_PIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = ROLL_BN_PIN | ROLL_CN_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    timerPWMadvancedConfig(TIM8);

    TIM8->CNT = 333;

    setupPWMIrq(TIM8_UP_IRQn);

    __disable_irq();
    {
        vu32 *tim8Enable = BB_PERIPH_ADDR(&(TIM8->CR1), 0);
        *tim8Enable = 1;
        TIM_CtrlPWMOutputs(TIM8, ENABLE);
    }
    __enable_irq();

    ///////////////////////////////////
    // Pitch PWM Timer Initialization here

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

    GPIO_InitStructure.GPIO_Pin   = PITCH_A_PIN | PITCH_B_PIN | PITCH_C_PIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = PITCH_AN_PIN | PITCH_BN_PIN | PITCH_CN_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    timerPWMadvancedConfig(TIM1);

    TIM1->CNT = 667;

    setupPWMIrq(TIM1_UP_IRQn);

    __disable_irq();
    {
        vu32 *tim1Enable = BB_PERIPH_ADDR(&(TIM1->CR1), 0);
        *tim1Enable = 1;
        TIM_CtrlPWMOutputs(TIM1, ENABLE);
    }
    __enable_irq();

    ///////////////////////////////////

    // Yaw PWM Timers Initialization here

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

    GPIO_InitStructure.GPIO_Pin   = YAW_A_PIN | YAW_B_PIN | YAW_C_PIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = YAW_AN_PIN | YAW_BN_PIN | YAW_CN_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    timerPWMgeneralConfig(TIM5, TIM_OCPolarity_High);
    timerPWMgeneralConfig(TIM4, TIM_OCPolarity_Low);

    TIM5->CNT = 0;
    TIM4->CNT = 0;

    setupPWMIrq(TIM5_IRQn);

    __disable_irq();
    {
        vu32 *tim5Enable = BB_PERIPH_ADDR(&(TIM5->CR1), 0);
        vu32 *tim4Enable = BB_PERIPH_ADDR(&(TIM4->CR1), 0);

        *tim5Enable = 1;
        *tim4Enable = 1;
    }
    __enable_irq();

    ///////////////////////////////////

    pwmMotorDriverInitDone = true;
}

///////////////////////////////////////////////////////////////////////////////

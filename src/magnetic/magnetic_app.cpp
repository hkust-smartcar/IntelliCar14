/*
 * magnetic_app.cpp
 * Magnetic App
 *
 * Author:
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <syscall.h>

#include <math.h>

#include "magnetic/car.h"

#include "magnetic/magnetic_app.h"

#include "mini_common.h"

#include"hw_common.h"

#include"MK60_uart.h"

#include"MK60_adc.h"

#include"MK60_FTM.h"

#include <log.h>
#include <libutil/clock.h>
#include<stdlib.h>

/*shit on 4/4/2014*/
volatile int LEFT_SENSOR_VALUE,RIGHT_SENSOR_VALUE,DELTA_SENSOR_VALUE_ADJ,DELTA_SENSOR_VALUE_PREV = 0,DELTA_SENSOR_VALUE;

volatile int  MID_SENSOR_VALUE;

volatile int angle,err,err_prev,err_sum;

volatile int spdr=3000,spdl=3000, temp;

volatile float testing_p = 0.9;

uint16 LEFT_SENSOR_VALUE_PREV,RIGHT_SENSOR_VALUE_PREV;

float pos_kp=0.95, pos_kd=1;

int no1;

float angle_adj;

/*shit on 4/4/2014 ends here*/

namespace magnetic
{

MagneticApp *MagneticApp::m_instance = nullptr;

MagneticApp::MagneticApp()
{
	m_instance = this;
}

MagneticApp::~MagneticApp()
{
	m_instance = nullptr;
}

void AdjustSpd()
{
if (DELTA_SENSOR_VALUE_ADJ < -90)
	(spdr=spdr+20);
	(spdl=spdl-30);
if (DELTA_SENSOR_VALUE_ADJ > 90)
(spdr=spdr-30);
(spdl=spdl+20);
if (DELTA_SENSOR_VALUE_ADJ > -89 && DELTA_SENSOR_VALUE_ADJ < 89 )
(spdr=2700,spdl=2700);

}

void InitAllShit()
{
	adc_init(ADC0_SE14);
	adc_init(ADC0_SE15);
	adc_init(ADC0_SE4b);
}

void CaluServoPercentage()
{
	if(DELTA_SENSOR_VALUE>-4500 && DELTA_SENSOR_VALUE < 4500)
		DELTA_SENSOR_VALUE_ADJ =0;
		else
	DELTA_SENSOR_VALUE_ADJ =pos_kp*( DELTA_SENSOR_VALUE)/72;// + pos_kd * err/400

	if (DELTA_SENSOR_VALUE_ADJ>137)
		(DELTA_SENSOR_VALUE_ADJ =137);
	if (DELTA_SENSOR_VALUE_ADJ <-137)
		(DELTA_SENSOR_VALUE_ADJ =-137);
}
void GetSensorValue()
{
	uint16 LEFT_SENSOR_VALUE, RIGHT_SENSOR_VALUE;//, MID_SENSOR_VALUE;
	//float theta;
	LEFT_SENSOR_VALUE = adc_once(ADC0_SE14,ADC_16bit);
	RIGHT_SENSOR_VALUE = adc_once(ADC0_SE15,ADC_16bit);
	MID_SENSOR_VALUE = adc_once(ADC0_SE4b,ADC_16bit);

	LEFT_SENSOR_VALUE= (round((LEFT_SENSOR_VALUE+2120)/250))*250;
	RIGHT_SENSOR_VALUE= (round(RIGHT_SENSOR_VALUE/250))*250;
	if (LEFT_SENSOR_VALUE > 65000)
	(LEFT_SENSOR_VALUE = 0);
	//theta= atan(MID_SENSOR_VALUE/(LEFT_SENSOR_VALUE-2120));
	//LEFT_SENSOR_VALUE=(LEFT_SENSOR_VALUE+2120)*cos(atan(MID_SENSOR_VALUE/(LEFT_SENSOR_VALUE+2120)));
	//RIGHT_SENSOR_VALUE=RIGHT_SENSOR_VALUE*cos(atan((RIGHT_SENSOR_VALUE)/MID_SENSOR_VALUE));
	DELTA_SENSOR_VALUE = (LEFT_SENSOR_VALUE - RIGHT_SENSOR_VALUE);
	LOG_W("%d",DELTA_SENSOR_VALUE);
	//no1=DELTA_SENSOR_VALUE_PREV-DELTA_SENSOR_VALUE;
}

void MagneticApp::Run()
{
	__g_fwrite_handler = FwriteHandler;
	libutil::Clock::Init();
	
	InitAllShit();
	while (true)
	{
		GetSensorValue();
		m_car.SetTurning(0);
		CaluServoPercentage();
		m_car.SetTurning(DELTA_SENSOR_VALUE_ADJ);
		m_car.SetMotorDirection(0);
		AdjustSpd();
		m_car.SetMotorPowerLeft(spdl);
		m_car.SetMotorPowerRight(spdr);
		err_sum=err+err_prev;
		//err= abs(DELTA_SENSOR_VALUE)/9648;
		GetSensorValue();
		DELTA_SENSOR_VALUE_PREV=DELTA_SENSOR_VALUE;
		err = err_prev;

	}

}

int MagneticApp::FwriteHandler(int, char *ptr, int len)
{
	if (m_instance)
	{
		m_instance->m_car.UartSendBuffer((const uint8_t*)ptr, len);
	}
	return len;

}

}

/*
 * magnetic_app.cpp
 * Magnetic App
 *
 * Author:
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <syscall.h>

#include <cstdio>

#include <math.h>

#include "magnetic/car.h"

#include "magnetic/magnetic_app.h"

#include "mini_common.h"

#include"hw_common.h"

#include"MK60_uart.h"

#include"MK60_adc.h"

#include"MK60_FTM.h"

#include <libutil/string.h>

#include <log.h>
#include <libutil/clock.h>
#include <stdlib.h>
#include <mini_common.h>
#include <hw_common.h>
#include <MK60_pit.h>
#include <vectors.h>

using namespace std;

/*shit on 4/4/2014*/
volatile int LEFT_SENSOR_VALUE,RIGHT_SENSOR_VALUE,DELTA_SENSOR_VALUE_ADJ,DELTA_SENSOR_VALUE_PREV = 0,DELTA_SENSOR_VALUE;

volatile int  MID_SENSOR_VALUE;

volatile int angle,err,err_prev,err_sum;

volatile int spd=1800;

volatile int pidspd=0,spdr=0,spdl=0, temp;

volatile int differential = 70;

volatile float testing_p = 0.9;

volatile float encoder=0;

volatile int encodercountl=0,encodercountr=0;

volatile int p=350; //speed control pid


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

/*void AdjustSpd()
{
spdr=spdr*dp;
if (DELTA_SENSOR_VALUE_ADJ < -90)
	(spdr=spd/(differential/100.0));
	(spdl=spd*(differential/100.0));
if (DELTA_SENSOR_VALUE_ADJ > 90)
(spdr=spd*(differential/100.0));
(spdl=spd/(differential/100.0));
if (DELTA_SENSOR_VALUE_ADJ > -89 && DELTA_SENSOR_VALUE_ADJ < 89 )
(spdr=spd,spdl=spd);

if (spdl>8000)  //motor protection speed <8000
	(spdl=8000);
if (spdr>8000)
	(spdl=8000);
}*/

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


__ISR void MagneticApp::Pit3Handler() //encoder + speed control pid
{
	m_instance->m_car.UpdateEncoder(0);
	m_instance->m_car.UpdateEncoder(1);
	encodercountl = m_instance->m_car.GetEncoderCount(0);
	encodercountr = m_instance->m_car.GetEncoderCount(1);


    spdl=(spd*(1+p*(encoder-encodercountl)/encoder/100));
    spdr=(spd*(1+p*(encoder-encodercountr)/encoder/100));

	m_instance->m_car.UartSendStr(libutil::String::Format("encoderl: %d, encoderr: %d", encodercountl, encodercountr).c_str());
	m_instance->m_car.UartSendStr(libutil::String::Format("spdl: %d, spdr: %d\n", spdl, spdr).c_str());

	if (DELTA_SENSOR_VALUE_ADJ > -89 && DELTA_SENSOR_VALUE_ADJ < 89 )
		(spdr=spdr,spdl=spdl);
	else if (DELTA_SENSOR_VALUE_ADJ < -90)
		{spdr=spdr/(differential/100.0);
		spdl=spdl*(differential/100.0);}
	else
		{spdr=spdr*(differential/100.0);
		spdl=spdl/(differential/100.0);}


	if (spdl<0)
		(spdl=0);
	if (spdr<0)
		(spdr=0);
	if (spdl>8000)  //motor protection speed <8000
		(spdl=8000);
	if (spdr>8000)
		(spdl=8000);

	m_instance->m_car.SetMotorPowerLeft(spdl);
	m_instance->m_car.SetMotorPowerRight(spdr);

	PIT_Flag_Clear(PIT3);
}


void MagneticApp::Run()
{
	__g_fwrite_handler = FwriteHandler;
	libutil::Clock::Init();
	
	InitAllShit();


	encoder=0.046*spd-45;
    SetIsr(PIT3_VECTORn, Pit3Handler);
    pit_init_ms(PIT3, 20);
    EnableIsr(PIT3_VECTORn);
	while (true)
	{
		GetSensorValue();
		m_car.SetTurning(0);
		CaluServoPercentage();
		m_car.SetTurning(DELTA_SENSOR_VALUE_ADJ);
		m_car.SetMotorDirection(0);
		//AdjustSpd();
		//m_car.SetMotorPowerLeft(spdl);
		//m_car.SetMotorPowerRight(spdr);
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
		//m_instance->m_car.UartSendBuffer((const uint8_t*)ptr, len);
	}
	return len;

}

}

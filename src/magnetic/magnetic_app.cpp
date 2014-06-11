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

#include<libutil/kalman_filter.h>

#include<libutil/clock.h>

using libutil::Clock;

using namespace std;

/*shit on 4/4/2014*/
volatile int RIGHT_SENSOR_VALUE_Y,LEFT_SENSOR_VALUE_X,RIGHT_SENSOR_VALUE_X;

volatile int LEFT_SENSOR_VALUE_Y_prev,LEFT_SENSOR_VALUE_X_prev,RIGHT_SENSOR_VALUE_X_prev;

volatile int LEFT_SENSOR_VALUE_BACK,RIGHT_SENSOR_VALUE_BACK;

int LEFT_SENSOR_VALUE_Y_filtered,LEFT_SENSOR_VALUE_Y;

int DELTA_SENSOR_VALUE,DELTA_SENSOR_VALUE_ADJ,DELTA_SENSOR_VALUE_ADJ_prev;

volatile int  MID_SENSOR_VALUE;

volatile int LEFT_FINAL,RIGHT_FINAL;

volatile float encoder,encoderl,encoderr;

volatile int angle,err,err_prev,err_sum;

volatile int spdr,spdl, temp;

int spd=1200;

#define differential 70

long int LEFT_SENSOR_VALUE_PREV,RIGHT_SENSOR_VALUE_PREV;

float pos_kp=0.6, pos_kd=1;

int no1;

float angle_adj;

//volatile int spd=2500;     //speed

//volatile int pidspd=0,spdr=spdl=2500, temp;


volatile float testing_p = 0.9;


volatile int encodercountl=0,encodercountr=0;

volatile int p=100; //speed control pid

#define K_ID 0

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


void MagneticApp::InitAll()
{
	adc_init(ADC0_SE10);
	adc_init(ADC0_SE14);
	adc_init(ADC1_SE4a);
	adc_init(ADC1_SE17);
	//adc_init(ADC1_SE5b);
}

void MagneticApp::CaluServoPercentage()
{
	/*if(DELTA_SENSOR_VALUE>-4500 && DELTA_SENSOR_VALUE < 4500)
		DELTA_SENSOR_VALUE_ADJ =0;
		else
		*/
	DELTA_SENSOR_VALUE_ADJ =pos_kp*(DELTA_SENSOR_VALUE)/72;// + pos_kd * err/400

	if(DELTA_SENSOR_VALUE_ADJ>-25&&DELTA_SENSOR_VALUE_ADJ<25)
		(DELTA_SENSOR_VALUE=0);
	if (DELTA_SENSOR_VALUE_ADJ>107)
		(DELTA_SENSOR_VALUE_ADJ =137);
	if (DELTA_SENSOR_VALUE_ADJ <-107)
		(DELTA_SENSOR_VALUE_ADJ =-137);
	int difference3 = DELTA_SENSOR_VALUE_ADJ_prev-DELTA_SENSOR_VALUE_ADJ;
	//if(difference3>-25&&difference3<25)
		//(DELTA_SENSOR_VALUE_ADJ=0);
	if (LEFT_SENSOR_VALUE_Y>1000)
	DELTA_SENSOR_VALUE_ADJ=DELTA_SENSOR_VALUE_ADJ*1.8;
	if(LEFT_SENSOR_VALUE_Y>1000&&DELTA_SENSOR_VALUE<1500&&DELTA_SENSOR_VALUE>-1500)
		DELTA_SENSOR_VALUE_ADJ=DELTA_SENSOR_VALUE_ADJ_prev;
	//LOG_W("%d",DELTA_SENSOR_VALUE_ADJ);

}

__ISR void MagneticApp::Pit1Handler()
{


	PIT_Flag_Clear(PIT1);

}
__ISR void MagneticApp::Pit3Handler() //encoder + speed control pid
{

	m_instance->m_car.UpdateEncoder();
	encodercountl = m_instance->m_car.GetEncoderCount(0);
	encodercountr = m_instance->m_car.GetEncoderCount(1);


	/*if (DELTA_SENSOR_VALUE_ADJ > -89 && DELTA_SENSOR_VALUE_ADJ < 89 )
		(encoder=encoderr,encoder=encoderl);
	else if (DELTA_SENSOR_VALUE_ADJ < -90)
		{encoderr=encoder/(differential/100.0);
		encoder=encoder*(differential/100.0);}
	else
		{encoderr=encoder*(differential/100.0);
		encoderl=encoder/(differential/100.0);}
		*/
	if(DELTA_SENSOR_VALUE_ADJ>60||DELTA_SENSOR_VALUE_ADJ<-60)
			spd= 2000;

    spdl=(spd*(1+p*(encoder-encodercountl)/encoder/100));
    spdr=(spd*(1+p*(encoder-encodercountr)/encoder/100));
    //LOG_W("%d,%d",spdl,spdr);
	//m_instance->m_car.UartSendStr(libutil::String::Format("encoderl: %d, encoderr: %d\n", encodercountl, encodercountr).c_str());
	//m_instance->m_car.UartSendStr(libutil::String::Format("spdl: %d, spdr: %d\n", spdl, spdr).c_str());

	if (spdl<0)
		(spdl=0);
	if (spdr<0)
		(spdr=0);

	if (spdl>8000)  //motor protection speed <8000
		(spdl=8000);
	if (spdr>8000)
		(spdl=8000);
	if (spdl>0 && encodercountl==0)
		(m_instance->m_car.SetMotorPowerLeft(0),m_instance->m_car.SetMotorPowerRight(0));

	else
	m_instance->m_car.SetMotorPowerLeft(spdl),m_instance->m_car.SetMotorPowerRight(spdr);

	PIT_Flag_Clear(PIT3);
}

void MagneticApp::GetSensorValue()

{
	int i,j,k,x,y,z;
	//float theta;
	x = adc_once(ADC0_SE10,ADC_16bit);//* 1.64416483;
	y = adc_once(ADC0_SE10,ADC_16bit);//* 1.64416483;
	z = adc_once(ADC0_SE10,ADC_16bit);//* 1.64416483;
	LEFT_SENSOR_VALUE_X=(x+y+z)/3;
	i = adc_once(ADC0_SE14,ADC_16bit);
	j = adc_once(ADC0_SE14,ADC_16bit);
	k = adc_once(ADC0_SE14,ADC_16bit);
	RIGHT_SENSOR_VALUE_X=(i+j+k)/3;
	LEFT_SENSOR_VALUE_Y=(adc_once(ADC1_SE4a,ADC_16bit))/2;
	RIGHT_SENSOR_VALUE_Y = adc_once(ADC1_SE17,ADC_16bit)/2;
	//LEFT_SENSOR_VALUE_X= (round((LEFT_SENSOR_VALUE_X)/150))*150;
	//RIGHT_SENSOR_VALUE_X= (round(RIGHT_SENSOR_VALUE_X/150))*150;
	LEFT_FINAL=((LEFT_SENSOR_VALUE_Y*LEFT_SENSOR_VALUE_Y)+LEFT_SENSOR_VALUE_X*LEFT_SENSOR_VALUE_X)/10000;
	LEFT_FINAL=sqrt(LEFT_FINAL)*100;
	RIGHT_FINAL=((RIGHT_SENSOR_VALUE_Y*RIGHT_SENSOR_VALUE_Y)+RIGHT_SENSOR_VALUE_X*RIGHT_SENSOR_VALUE_X)/10000;
	RIGHT_FINAL=sqrt(RIGHT_FINAL)*100;
	//RIGHT_SENSOR_VALUE_BACK=(round((RIGHT_SENSOR_VALUE_BACK)/250))*250;
	//theta= atan(MID_SENSOR_VALUE/(LEFT_SENSOR_VALUE-2120));
	//LEFT_SENSOR_VALUE=(LEFT_SENSOR_VALUE+2120)*cos(atan(MID_SENSOR_VALUE/(LEFT_SENSOR_VALUE+2120)));
	//RIGHT_SENSOR_VALUE=RIGHT_SENSOR_VALUE*cos(atan((RIGHT_SENSOR_VALUE)/MID_SENSOR_VALUE));
	DELTA_SENSOR_VALUE = (LEFT_FINAL - RIGHT_FINAL);
	//no1=DELTA_SENSOR_VALUE_PREV-DELTA_SENSOR_VALUE;
	//if (no1<-300&&(DELTA_SENSOR_VALUE>-5000||DELTA_SENSOR_VALUE<5000))
		//(DELTA_SENSOR_VALUE=DELTA_SENSOR_VALUE_PREV);
}

void NoiseFiltering()
{
	int difference1 = LEFT_SENSOR_VALUE_Y- LEFT_SENSOR_VALUE_Y_prev;
	if (difference1 >20)
	LEFT_SENSOR_VALUE_Y= LEFT_SENSOR_VALUE_Y_prev +5;
	if (difference1 <-20 && difference1<0)
	LEFT_SENSOR_VALUE_Y= LEFT_SENSOR_VALUE_Y_prev -5;
	//m_abc=libutil::KalmanFilter(0.005f, 0.05f,adc_once(ADC0_SE4b,ADC_16bit),1.0f);
	//int difference2 = RIGHT_SENSOR_VALUE_X- RIGHT_SENSOR_VALUE_X_prev;
		//if (difference2 >20)
			//RIGHT_SENSOR_VALUE_X= RIGHT_SENSOR_VALUE_X_prev +5;
		//if (difference2 <-20)
			//RIGHT_SENSOR_VALUE_X= RIGHT_SENSOR_VALUE_X_prev -5;

		//int difference4 = LEFT_SENSOR_VALUE_X- LEFT_SENSOR_VALUE_X_prev;
				//if (difference4 >20)
					//LEFT_SENSOR_VALUE_X= LEFT_SENSOR_VALUE_X_prev +5;
				//if (difference4 <-20)
					//LEFT_SENSOR_VALUE_X= LEFT_SENSOR_VALUE_X_prev -5;
		//LOG_W("%d,%d",LEFT_SENSOR_VALUE_Y,LEFT_SENSOR_VALUE_Y_prev);
}

void AdjustSpd()
{
	if(DELTA_SENSOR_VALUE_ADJ>60)
		spdl=3200,spdr=1500;
	else if((DELTA_SENSOR_VALUE_ADJ<-60))
		spdl=1500,spdr=3200;
	else spdl=2500, spdr=2500;

}

void MagneticApp::Run()
{
	/*__g_fwrite_handler = FwriteHandler;
	libutil::Clock::Init();
	encoder=0.046*spd-45;
	SetIsr(PIT3_VECTORn, Pit3Handler);
	pit_init_ms(PIT3, 30);
	EnableIsr(PIT3_VECTORn);
*/
/*
	SetIsr(PIT1_VECTORn, Pit1Handler);
	pit_init_ms(PIT1, 30);
	EnableIsr(PIT1_VECTORn);
*/
		while (true)
	{
		InitAll();
		GetSensorValue();
		//NoiseFiltering();
		iprintf("abc/n");
		CaluServoPercentage();
		m_car.SetTurning(DELTA_SENSOR_VALUE_ADJ);
		m_car.SetMotorDirection(0);
		//AdjustSpd();
		//err= abs(DELTA_SENSOR_VALUE)/9648;
		err = err_prev;
		//LOG_W("%d,%d,%d,%d",DELTA_SENSOR_VALUE_ADJ,LEFT_SENSOR_VALUE_Y,LEFT_SENSOR_VALUE_X,RIGHT_SENSOR_VALUE_X);//,LEFT_SENSOR_VALUE_Y);
		LOG_W("%d,%d,%d",LEFT_FINAL,RIGHT_SENSOR_VALUE_X,RIGHT_SENSOR_VALUE_Y);
		DELAY_MS(60);
		LEFT_SENSOR_VALUE_Y_prev=LEFT_SENSOR_VALUE_Y;
		RIGHT_SENSOR_VALUE_X_prev=RIGHT_SENSOR_VALUE_X;
		DELTA_SENSOR_VALUE_ADJ=DELTA_SENSOR_VALUE_ADJ_prev;
		m_instance->m_car.SetMotorPowerLeft(2700),m_instance->m_car.SetMotorPowerRight(2700);
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

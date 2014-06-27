/*
 * magnetic_app.cpp
 * Magnetic App
 *
 * Author:
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <syscall.h>

#include <cstdint>

#include <cstdlib>

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

#include "MK60_gpio.h"

#include <stdio.h>

#include <libutil/misc.h>

#include <libutil/string.h>

using libutil::Clock;

using namespace std;

/*shit on 4/4/2014*/
volatile int RIGHT_SENSOR_VALUE_Y,LEFT_SENSOR_VALUE_X,RIGHT_SENSOR_VALUE_X;

volatile int slope;

volatile int LEFT_SENSOR_VALUE_Y_prev,LEFT_SENSOR_VALUE_X_prev,RIGHT_SENSOR_VALUE_X_prev;

volatile int LEFT_SENSOR_VALUE_BACK,RIGHT_SENSOR_VALUE_BACK;

volatile int LEFT_SENSOR_VALUE_Y_filtered,LEFT_SENSOR_VALUE_Y;

volatile int DELTA_SENSOR_VALUE,DELTA_SENSOR_VALUE_ADJ,DELTA_SENSOR_VALUE_ADJ_prev, DELTA_SENSOR_VALUE_prev;

volatile int  MID_SENSOR_VALUE;

volatile int LEFT_FINAL,RIGHT_FINAL;

volatile int encoder;

volatile int angle,err,err_prev,err_sum;

volatile int  temp,spd_temp;

volatile int con1 = 13000;

int i=0, j=1;

volatile int spd;

float differential=1.5;

int mean_delta;

long int LEFT_SENSOR_VALUE_PREV,RIGHT_SENSOR_VALUE_PREV;

//float pos_kp=0.8, pos_kd=10;

//float pos_kp=1.75, pos_kd=3.84375*2;
float pos_kp, pos_kd;//pos_kd=0.104375;
volatile float spdl_err, spdr_err, spdl_err_prev, spdr_err_prev ;

//testing value pos_kp=107, pos_kd=0.2675;

//volatile int spd=2500;     //speed

//volatile int pidspd=0,spdr=spdl=2500, temp;


volatile float testing_p = 0.9;


volatile int spdl,spdr,encodercountl,encodercountr;

volatile float spd_kp=1.0,spd_ki=0,spd_kd=0.0; //speed control pid

#define K_ID 0

bool intrack;
Clock::ClockInt prev_time= Clock::Time();

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
	adc_init(ADC0_SE17);
	//adc_init(ADC1_SE5b);
}

void MagneticApp::CaluServoPercentage()
{
	//const Clock::ClockInt prev_time;
	const Clock::ClockInt time = Clock::Time();
	const Clock::ClockInt time_diff = Clock::TimeDiff(time, prev_time);
	//m_car.LcdSetRow(4);
	//m_car.LcdPrintString(libutil::String::Format("timediff: %d\n", time_diff).c_str(), 0xFFFF);
	//pos_kp=0.5,pos_kd=4;
	//if (LEFT_FINAL<10000&&RIGHT_FINAL<10000) pos_kp = 0.8,pos_kd=0.4;
	pos_kp=1.19, pos_kd=1.938/2;//0.3768/1000000;
	if (LEFT_FINAL >8000 && RIGHT_FINAL>8000&&DELTA_SENSOR_VALUE<2000) pos_kp=pos_kp*3/5,pos_kd=pos_kd*4/5 ;
	if (LEFT_FINAL<RIGHT_FINAL)(pos_kp=pos_kp*1.35);
	//if(DELTA_SENSOR_VALUE>0)(DELTA_SENSOR_VALUE=DELTA_SENSOR_VALUE*0.83333);
	//if (slope >200) (slope=200);
	//if (slope <-200) (slope=-200);
	//LOG_W("%d",err);
	DELTA_SENSOR_VALUE_ADJ =pos_kp*(DELTA_SENSOR_VALUE)/82+ pos_kd * slope/(time_diff*80)+20 ;
	//if(DELTA_SENSOR_VALUE_ADJ<-65) DELTA_SENSOR_VALUE_ADJ=DELTA_SENSOR_VALUE_ADJ-15;
	//if(DELTA_SENSOR_VALUE_ADJ>65) DELTA_SENSOR_VALUE_ADJ=DELTA_SENSOR_VALUE_ADJ+15;
	if(DELTA_SENSOR_VALUE_ADJ>-20&&DELTA_SENSOR_VALUE_ADJ<20)
		(DELTA_SENSOR_VALUE=0);
	if (DELTA_SENSOR_VALUE_ADJ>100)
		(DELTA_SENSOR_VALUE_ADJ =100);
	if (DELTA_SENSOR_VALUE_ADJ <-134)
		(DELTA_SENSOR_VALUE_ADJ =-134);
	//if(LEFT_SENSOR_VALUE_Y>1000&&DELTA_SENSOR_VALUE<1500&&DELTA_SENSOR_VALUE>-1500)
		//DELTA_SENSOR_VALUE_ADJ=DELTA_SENSOR_VALUE_ADJ_prev;
	//LOG_W("%d",DELTA_SENSOR_VALUE_ADJ);
	//if ((DELTA_SENSOR_VALUE_ADJ>2000)&&(LEFT_SENSOR_VALUE_Y>6000)) (DELTA_SENSOR_VALUE_ADJ=DELTA_SENSOR_VALUE_ADJ+30);
	//if ((DELTA_SENSOR_VALUE_ADJ<-2000)&&(RIGHT_SENSOR_VAL > UE_Y>000)) (DELTA_SENSOR_VALUE_ADJ=DELTA_SENSOR_VALUE_ADJ-30);
	//if ((DELTA_SENSOR_VALUE_ADJ_prev> 80) && (DELTA_SENSOR_VALUE_ADJ> 80)) (pos_kp=1.9);
	//if ((DELTA_SENSOR_VALUE_ADJ_prev<-80) && (DELTA_SENSOR_VALUE_ADJ<-80)) (pos_kp=1.9);
	/*bool IsLeftMid=0;
	if (LEFT_FINAL>8000) IsLeftMid=1;
	bool IsRightMid=0;
*/
	prev_time = time;

}

void MagneticApp::DirectionControl()
{
		InitAll();
			LEFT_SENSOR_VALUE_X= (adc_once(ADC0_SE10,ADC_16bit) -800)*1.32654;
			if (LEFT_SENSOR_VALUE_X<4500)(LEFT_SENSOR_VALUE_X=LEFT_SENSOR_VALUE_X/2);
			RIGHT_SENSOR_VALUE_X=(adc_once(ADC0_SE14,ADC_16bit)-1300)*1.184849;
			//LEFT_SENSOR_VALUE_Y=(adc_once(ADC1_SE4a,ADC_16bit))*1.325+800;
			LEFT_SENSOR_VALUE_Y=(adc_once(ADC1_SE4a,ADC_16bit))-800;
			//if (LEFT_SENSOR_VALUE_Y <9500&&LEFT_SENSOR_VALUE_Y >4000)(LEFT_SENSOR_VALUE_Y=LEFT_SENSOR_VALUE_Y/2.89);
			//RIGHT_SENSOR_VALUE_Y = adc_once(ADC0_SE17,ADC_16bit)*0.841471413;
			RIGHT_SENSOR_VALUE_Y = adc_once(ADC0_SE17,ADC_16bit)-1050;
			if (RIGHT_SENSOR_VALUE_Y <7000 )(RIGHT_SENSOR_VALUE_Y=RIGHT_SENSOR_VALUE_Y/2.2);
			//if (RIGHT_SENSOR_VALUE_Y<8500)(RIGHT_SENSOR_VALUE_Y=RIGHT_SENSOR_VALUE_Y/8);
			//if (RIGHT_SENSOR_VALUE_Y <9500&&RIGHT_SENSOR_VALUE_Y >4000)(RIGHT_SENSOR_VALUE_Y=RIGHT_SENSOR_VALUE_Y/2.89);
			//if (LEFT_SENSOR_VALUE_Y <9500&&LEFT_SENSOR_VALUE_Y >6000)(LEFT_SENSOR_VALUE_Y=LEFT_SENSOR_VALUE_Y/2.89);
			//LEFT_SENSOR_VALUE_X= (round((LEFT_SENSOR_VALUE_X)/150))*150;
			//RIGHT_SENSOR_VALUE_X= (round(RIGHT_SENSOR_VALUE_X/150))*150;
			LEFT_FINAL=((LEFT_SENSOR_VALUE_Y*LEFT_SENSOR_VALUE_Y)+LEFT_SENSOR_VALUE_X*LEFT_SENSOR_VALUE_X)/10000;
			LEFT_FINAL=sqrt(LEFT_FINAL)*100;
			RIGHT_FINAL=((RIGHT_SENSOR_VALUE_Y*RIGHT_SENSOR_VALUE_Y)+RIGHT_SENSOR_VALUE_X*RIGHT_SENSOR_VALUE_X)/10000;
			RIGHT_FINAL=sqrt(RIGHT_FINAL)*100;
			//RI1GHT_SENSOR_VALUE_BACK=(round((RIGHT_SENSOR_VALUE_BACK)/250))*250;
			//theta= atan(MID_SENSOR_VALUE/(LEFT_SENSOR_VALUE-2120));
			//LEFT_SENSOR_VALUE=(LEFT_SENSOR_VALUE+2120)*cos(atan(MID_SENSOR_VALUE/(LEFT_SENSOR_VALUE+2120)));
			//RIGHT_SENSOR_VALUE=RIGHT_SENSOR_VALUE*cos(atan((RIGHT_SENSOR_VALUE)/MID_SENSOR_VALUE));
			int temp=(LEFT_FINAL - RIGHT_FINAL);
			DELTA_SENSOR_VALUE =20000*temp/(LEFT_FINAL + RIGHT_FINAL);
			if (intrack==0)(DELTA_SENSOR_VALUE=0);
			//LOG_W("%d,%d",DELTA_SENSOR_VALUE,RIGHT_FINAL);
			slope =(DELTA_SENSOR_VALUE-DELTA_SENSOR_VALUE_prev);

			DELTA_SENSOR_VALUE_prev=temp;
}

void MagneticApp::SpdCtrlpass() //encoder + speed control pid
{
	spd=2400;
	bool motor_dir=0;
	//if (DELTA_SENSOR_VALUE_ADJ>50||DELTA_SENSOR_VALUE_ADJ<-50)(spd=2500);
	const Clock::ClockInt time = Clock::Time();
	const Clock::ClockInt time_diff = Clock::TimeDiff(time, prev_time);
	m_instance->m_car.UpdateEncoder();
	encodercountr = m_instance->m_car.GetEncoderCount(0);
	encodercountr=encodercountr*28.2352941176;
	//encodercountl = m_instance->m_car.GetEncoderCount(1);
	//encodercountl=encodercountl*69;
	//encodercountl=77*encodercountl/72;
	/*if (DELTA_SENSOR_VALUE_ADJ > 80 )
		(spdl=spdl/differential,spdr=spdr*differential);
	else if (DELTA_SENSOR_VALUE_ADJ < -80)
		{spdl=spdl*differential,spdr=spdr/differential;}
*/
	//if(DELTA_SENSOR_VALUE_ADJ < -60)(spdl=spdl*2.1);
	//2if(DELTA_SENSOR_VALUE_ADJ >5)(spdr=spdr*2.1);
	//if(DELTA_SENSOR_VALUE_ADJ>60||DELTA_SENSOR_VALUE_ADJ<-60)
			//spd= spd*0.75;
	 m_instance->m_car.SetMotorLeftDirection(motor_dir);
	 m_instance->m_car.SetMotorRightDirection(motor_dir);
	//if (motor_dir==1) (encodercountr=-1*encodercountr);
	//spdl_prop=spd_kp*((spd-encodercountr)/spd);
	int spdr_prop=2400-encodercountr;
    spdr=(2400*(1+(1.8*(2400-encodercountr)/2400)));//+time_diff/2*spd_ki*(spdl_err_prev+spdl_err);
    if (spdr<0)(motor_dir=1,spdr=abs(spdr));
    else(motor_dir=0);
    //spdr=(spd+(spd*(0.8*((spd-encodercountr)/spd))));//+time_diff/2*spd_ki*(spdr_err_prev+spdr_err);
	//m_instance->m_car.UartSendStr(libutil::String::Format("spdl: %d, spdr: %d\n", spdl, spdr).c_str());
    //LOG_W("%d,%d",encodercountl,encodercountr)
    //printf("%d\n",encodercountl);
	//if (spdr<0)(m_instance->m_car.SetMotorRightDirection(1),spdr=abs(spdr));


	if ((LEFT_FINAL<500&&RIGHT_FINAL<500)||(encodercountr==1))(intrack= false);
	if (intrack == false) (spdl=0,spdr=0);

	//m_instance->m_car.SetMotorPowerLeft(libutil::Clamp<int>(-10000, 2500, 10000)),m_instance->m_car.SetMotorPowerRight((libutil::Clamp<int>(-10000, 2500, 10000)));
	m_instance->m_car.SetMotorPowerLeft(spdr),m_instance->m_car.SetMotorPowerRight(spdr);

	m_car.LcdSetRow(3);
	m_car.LcdPrintString(libutil::String::Format("L: %d\n", encodercountr).c_str(), 0xFFFF);
	m_car.LcdSetRow(4);
	m_car.LcdPrintString(libutil::String::Format("L: %d\n", spdr_prop).c_str(), 0xFFFF);
	prev_time = time;
	spdr_err_prev=spdr_err;
	spdl_err_prev=spdl_err;
	DELAY_MS(10);
}
void MagneticApp::ServoFTMHandler()
{

}

void MagneticApp::Run()
{

	DELAY_MS(1500);
	InitAll();
LEFT_SENSOR_VALUE_X=adc_once(ADC0_SE10,ADC_16bit)* 1.09610989;
RIGHT_SENSOR_VALUE_X=adc_once(ADC0_SE14,ADC_16bit)*1;
if (LEFT_SENSOR_VALUE_X>2500&&RIGHT_SENSOR_VALUE_X>2500) (intrack=true);
	__g_fwrite_handler = FwriteHandler;
	libutil::Clock::Init();

	/*SetIsr(PIT3_VECTORn, ServoFTMHandler);
	pit_init_ms(PIT3, 5);
	EnableIsr(PIT3_VECTORn);*/
//intrack==
		while (true)
	{
		InitAll();
		DirectionControl();
		CaluServoPercentage();
		m_car.SetTurning(DELTA_SENSOR_VALUE_ADJ);
		//LOG_W("%d,%d,%d,%d",DELTA_SENSOR_VALUE_ADJ,LEFT_SENSOR_VALUE_Y,LEFT_SENSOR_VALUE_X,RIGHT_SENSOR_VALUE_X);//,LEFT_SENSOR_VALUE_Y);
		//printf("%d,%d,%d,%d\n",LEFT_SENSOR_VALUE_X,LEFT_SENSOR_VALUE_Y,RIGHT_SENSOR_VALUE_X,RIGHT_SENSOR_VALUE_Y);
		//printf("%d\n",LEFT_SENSOR_VALUE_Y);
		DELAY_MS(10);
		SpdCtrlpass();
		//if ((DELTA_SENSOR_VALUE_ADJ_prev>0&&DELTA_SENSOR_VALUE_ADJ>0)||(DELTA_SENSOR_VALUE_ADJ_prev<0&&DELTA_SENSOR_VALUE_ADJ<0))(mean_delta=(DELTA_SENSOR_VALUE_ADJ_prev+DELTA_SENSOR_VALUE_ADJ))/2;
		////printf("%d,%d\n",LEFT_FINAL,RIGHT_FINAL);
		LEFT_SENSOR_VALUE_Y_prev=LEFT_SENSOR_VALUE_Y;
		RIGHT_SENSOR_VALUE_X_prev=RIGHT_SENSOR_VALUE_X;
		//DELTA_SENSOR_VALUE_ADJ_prev=DELTA_SENSOR_VALUE_ADJ;
		m_car.LcdSetRow(1);
		m_car.LcdPrintString(libutil::String::Format("L: %d,%d\n",LEFT_SENSOR_VALUE_X,LEFT_SENSOR_VALUE_Y).c_str(), 0xFFFF);
		m_car.LcdSetRow(2);
		m_car.LcdPrintString(libutil::String::Format("R: %d,%d\n",RIGHT_SENSOR_VALUE_X,RIGHT_SENSOR_VALUE_Y).c_str(), 0xFFFF);


		//DELAY_MS(300);
		//m_instance->m_car.SetMotorPowerLeft(1800),m_instance->m_car.SetMotorPowerRight(1800);
		if (LEFT_SENSOR_VALUE_X<2000&&RIGHT_SENSOR_VALUE_X<2000) (intrack=false);
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

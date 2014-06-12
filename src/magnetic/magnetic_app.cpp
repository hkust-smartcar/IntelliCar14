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

volatile int con1 = 13000;

int spd=2300;

#define differential 70

long int LEFT_SENSOR_VALUE_PREV,RIGHT_SENSOR_VALUE_PREV;

float pos_kp=0.7, pos_kd=1;

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
	adc_init(ADC0_SE17);
	//adc_init(ADC1_SE5b);
}

void MagneticApp::CaluServoPercentage()
{
	/*if(DELTA_SENSOR_VALUE>-4500 && DELTA_SENSOR_VALUE < 4500)
		DELTA_SENSOR_VALUE_ADJ =0;
		else
		*/
	DELTA_SENSOR_VALUE_ADJ =pos_kp*(DELTA_SENSOR_VALUE)/72;// + pos_kd * err/400

	if(DELTA_SENSOR_VALUE_ADJ>-15&&DELTA_SENSOR_VALUE_ADJ<15)
		(DELTA_SENSOR_VALUE=0);
	if (DELTA_SENSOR_VALUE_ADJ>107)
		(DELTA_SENSOR_VALUE_ADJ =137);
	if (DELTA_SENSOR_VALUE_ADJ <-107)
		(DELTA_SENSOR_VALUE_ADJ =-137);
	//int difference3 = DELTA_SENSOR_VALUE_ADJ_prev-DELTA_SENSOR_VALUE_ADJ;
	//if(difference3>-25&&difference3<25)
		//(DELTA_SENSOR_VALUE_ADJ=0);
	//if (DELTA_SENSOR_VALUE_ADJ>3000)
	//DELTA_SENSOR_VALUE_ADJ=DELTA_SENSOR_VALUE_ADJ*2.5;
	if(LEFT_SENSOR_VALUE_Y>1000&&DELTA_SENSOR_VALUE<1500&&DELTA_SENSOR_VALUE>-1500)
		DELTA_SENSOR_VALUE_ADJ=DELTA_SENSOR_VALUE_ADJ_prev;
	//LOG_W("%d",DELTA_SENSOR_VALUE_ADJ);
	//if ((DELTA_SENSOR_VALUE_ADJ>2000)&&(LEFT_SENSOR_VALUE_Y>6000)) (DELTA_SENSOR_VALUE_ADJ=DELTA_SENSOR_VALUE_ADJ+30);
	//if ((DELTA_SENSOR_VALUE_ADJ<-2000)&&(RIGHT_SENSOR_VAL > UE_Y>000)) (DELTA_SENSOR_VALUE_ADJ=DELTA_SENSOR_VALUE_ADJ-30);
	if ((DELTA_SENSOR_VALUE_ADJ_prev> 80) && (DELTA_SENSOR_VALUE_ADJ> 80)) (pos_kp=1.9);
	if ((DELTA_SENSOR_VALUE_ADJ_prev<-80) && (DELTA_SENSOR_VALUE_ADJ<-80)) (pos_kp=1.9);
	bool IsLeftMid=0;
	if (LEFT_FINAL>8000) IsLeftMid=1;
	/*bool IsRightMid=0;

	bool IsMid=0 ;
	if (IsLeftMid==1 && IsRightMid==1)(IsMid=1);

	if (RIGHT_FINAL>8000) IsRightMid=1;
	if ((DELTA_SENSOR_VALUE_ADJ_prev>40||DELTA_SENSOR_VALUE_ADJ_prev<-40)&&(IsMid=0))
	(DELTA_SENSOR_VALUE_ADJ=DELTA_SENSOR_VALUE_ADJ_prev*1.2);
	if (((DELTA_SENSOR_VALUE_ADJ*DELTA_SENSOR_VALUE_ADJ_prev)<-100)&&(IsMid=0))(DELTA_SENSOR_VALUE_ADJ=-1*DELTA_SENSOR_VALUE_ADJ);
*/
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
	encodercountr=1.14583333*encodercountr;

	/*if (DELTA_SENSOR_VALUE_ADJ > -70 && DELTA_SENSOR_VALUE_ADJ < 70 )
		(encoder=encoderr,encoder=encoderl);
	else if (DELTA_SENSOR_VALUE_ADJ < -90)
		{encoderr=encoder/(differential/100.0);
		encoder=encoder*(differential/100.0);}
	else
		{encoderr=encoder*(differential/100.0);
		encoderl=encoder/(differential/100.0);}
		*/
	if(DELTA_SENSOR_VALUE_ADJ < -90)(spdl*1.8);
	if(DELTA_SENSOR_VALUE_ADJ >90)(spdr*1.8);
	if(DELTA_SENSOR_VALUE_ADJ>60||DELTA_SENSOR_VALUE_ADJ<-60)
			spd= 2200;

    spdl=(spd*(1+p*(encoder-encodercountl)/encoder/100));
    spdr=(spd*(1+p*(encoder-encodercountr)/encoder/100));
    //LOG_W("%d,%d",spdl,spdr);
	//m_instance->m_car.UartSendStr("%d", encodercountl);
	//m_instance->m_car.UartSendStr(libutil::String::Format("spdl: %d, spdr: %d\n", spdl, spdr).c_str());
    //LOG_W("%d,%d",encodercountl,encodercountr);
	if (spdl<0)
		(spdl=0);
	if (spdr<0)
		(spdr=0);

	if (spdl>8000)  //motor protection speed <8000
		(spdl=8000);
	if (spdr>8000)
		(spdl=8000);
	//if (spdl>0 && encodercountl==0)
		//(m_instance->m_car.SetMotorPowerLeft(0),m_instance->m_car.SetMotorPowerRight(0));

	//else
	m_instance->m_car.SetMotorPowerLeft(spdl),m_instance->m_car.SetMotorPowerRight(spdr);

	PIT_Flag_Clear(PIT3);
}

void MagneticApp::GetSensorValue()

{
	int i,j,k,x,y,z;
	//float theta;
	x = adc_once(ADC0_SE10,ADC_16bit)* 1.09610989;
	y = adc_once(ADC0_SE10,ADC_16bit)*1.09610989;
	z = adc_once(ADC0_SE10,ADC_16bit) *1.09610989;
	LEFT_SENSOR_VALUE_X=(x+y+z)/3;
	i = adc_once(ADC0_SE14,ADC_16bit)*1;
	j = adc_once(ADC0_SE14,ADC_16bit)*1;
	k = adc_once(ADC0_SE14,ADC_16bit)*1;
	RIGHT_SENSOR_VALUE_X=(i+j+k)/3;
	if (DELTA_SENSOR_VALUE_ADJ>3000) RIGHT_SENSOR_VALUE_X=RIGHT_SENSOR_VALUE_X+3000;
	LEFT_SENSOR_VALUE_Y=(adc_once(ADC1_SE4a,ADC_16bit))/2+1000;
	RIGHT_SENSOR_VALUE_Y = adc_once(ADC0_SE17,ADC_16bit)/2;
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
	int temp = LEFT_FINAL - RIGHT_FINAL;
	if (LEFT_FINAL <9000 || RIGHT_FINAL<9000) con1 = 6500;
	DELTA_SENSOR_VALUE = (temp);

	//no1=DELTA_SENSOR_VALUE_PREV-DELTA_SENSOR_VALUE;
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
	__g_fwrite_handler = FwriteHandler;
	libutil::Clock::Init();


	encoder=0.046*spd-45;
	SetIsr(PIT3_VECTORn, Pit3Handler);
	pit_init_ms(PIT3, 30);
	EnableIsr(PIT3_VECTORn);
/*
	SetIsr(PIT1_VECTORn, Pit1Handler);
	pit_init_ms(PIT1, 30);
	EnableIsr(PIT1_VECTORn);
*/
		while (true)
	{
		InitAll();
		GetSensorValue();
		//iprintf("abc/n");
		CaluServoPercentage();
		m_car.SetTurning(DELTA_SENSOR_VALUE_ADJ);
		//m_car.SetTurning(0);
		m_car.SetMotorDirection(0);
		//AdjustSpd();
		//err= abs(DELTA_SENSOR_VALUE)/9648;
		err = err_prev;
		//LOG_W("%d,%d,%d,%d",DELTA_SENSOR_VALUE_ADJ,LEFT_SENSOR_VALUE_Y,LEFT_SENSOR_VALUE_X,RIGHT_SENSOR_VALUE_X);//,LEFT_SENSOR_VALUE_Y);
		LOG_W("%d,%d,%d,%d",LEFT_SENSOR_VALUE_X,LEFT_SENSOR_VALUE_Y,RIGHT_SENSOR_VALUE_X,RIGHT_SENSOR_VALUE_Y);
		DELAY_MS(40);
		LEFT_SENSOR_VALUE_Y_prev=LEFT_SENSOR_VALUE_Y;
		RIGHT_SENSOR_VALUE_X_prev=RIGHT_SENSOR_VALUE_X;
		DELTA_SENSOR_VALUE_ADJ_prev=DELTA_SENSOR_VALUE_ADJ;
		//m_instance->m_car.SetMotorPowerLeft(2400),m_instance->m_car.SetMotorPowerRight(2400);
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

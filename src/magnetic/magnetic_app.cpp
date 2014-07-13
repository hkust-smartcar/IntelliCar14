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

#include<libsc/k60/system.h>
#include<libsc/k60/timer.h>

#include "MK60_gpio.h"

#include <stdio.h>

#include <libutil/misc.h>

#include <libutil/string.h>

using namespace libsc::k60;

using namespace std;

/*shit on 4/4/2014*/
volatile int RIGHT_SENSOR_VALUE_Y,LEFT_SENSOR_VALUE_X,RIGHT_SENSOR_VALUE_X;

volatile int slope;

volatile int LEFT_SENSOR_VALUE_Y_prev,LEFT_SENSOR_VALUE_X_prev,RIGHT_SENSOR_VALUE_X_prev;


volatile int LEFT_SENSOR_VALUE_Y_filtered,LEFT_SENSOR_VALUE_Y;

volatile int DELTA_SENSOR_VALUE,DELTA_SENSOR_VALUE_ADJ,DELTA_SENSOR_VALUE_ADJ_prev, DELTA_SENSOR_VALUE_prev,DELTA_SENSOR_VALUE_RAW,DELTA_prev_turn,DELTA_deter_spd;

volatile float LEFT_FINAL,RIGHT_FINAL;
volatile float LEFT_FINAL_RAW,RIGHT_FINAL_RAW;

volatile int encoder;

volatile int temp_RAW, temp_RAW_prev;

volatile int angle,err,err_prev,err_sum;

volatile float temp,spd_temp,DELTA_VALUE_OFFSET;

volatile float sen_lx_pred,sen_ly_pred,sen_rx_pred,sen_ry_pred;
volatile float LEFT_SENSOR_VALUE_X_filter,LEFT_SENSOR_VALUE_Y_filter,RIGHT_SENSOR_VALUE_X_filter ,RIGHT_SENSOR_VALUE_Y_filter;

volatile int spd,encodercountr_prev;

//volatile int dip_sw_bin[4]={0,0,0,0};

volatile float LEFT_SENSOR_VALUE_X_filter_RAW, LEFT_SENSOR_VALUE_Y_filter_RAW, RIGHT_SENSOR_VALUE_X_filter_RAW,RIGHT_SENSOR_VALUE_Y_filter_RAW;

long int LEFT_SENSOR_VALUE_PREV,RIGHT_SENSOR_VALUE_PREV;
float pos_kp, pos_kd;//pos_kd=0.104375;
volatile int32_t spdl_err, spdr_err, spdl_err_prev, spdr_err_prev ;
int32_t total_error_l = 0, total_error_r = 0;

//testing value pos_kp=107, pos_kd=0.2675;

//volatile int spd=2500;     //speed

//volatile int pidspd=0,spdr=spdl=2500, temp;


volatile float testing_p = 0.9;

volatile uint32 sys_mode;

volatile int encodercountl,encodercountr;


volatile float spd_kp=5.0,spd_ki=0.0,spd_kd=0.0, spdl,spdr; //speed control pid

#define K_ID 0

volatile uint8_t state_flag = 0;
volatile uint8_t prev_state_flag = 0;
volatile uint8_t protected_count = 0;

bool intrack;
Timer::TimerInt prev_time= 0;

/*shit on 4/4/2014 ends here*/

namespace magnetic
{

MagneticApp *MagneticApp::m_instance = nullptr;

MagneticApp::MagneticApp()
		: m_MagneticSenlx_filter(0.0f, 0.0f, 0.0f, 1.0f), m_MagneticSenly_filter(0.0f, 0.0f, 0.0f, 1.0f),m_MagneticSenrx_filter(0.0f, 0.0f, 0.0f, 1.0f),m_MagneticSenry_filter(0.0f, 0.0f, 0.0f, 1.0f)
				//m_MagneticSen_filter(0.0f, 2617693.0f, 0.0f, 1.0f)
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
	gpio_init  (PTE9, GPI, 0);
	gpio_init  (PTE10, GPI, 0);
	gpio_init  (PTE11, GPI, 0);
	gpio_init  (PTE12, GPI, 0);

	m_MagneticSenlx_filter=libutil::KalmanFilter(0.0005f,5000.0f,adc_average(ADC0_SE10,ADC_16bit,100)*1.454545554,1.0f);//618203.071f
	m_MagneticSenly_filter=libutil::KalmanFilter(18000.0f,20000.0f,adc_average(ADC1_SE4a,ADC_16bit,100),1.0f);
	m_MagneticSenrx_filter=libutil::KalmanFilter(0.0005f,5000.0f,adc_average(ADC0_SE14,ADC_16bit,100)*1.5,1.0f);
	m_MagneticSenry_filter=libutil::KalmanFilter(18000.0f,20000.0f,adc_average(ADC0_SE17,ADC_16bit,100),1.0f);
	//pos_kp=0.22, pos_kd=2.2;
	//adc_init(ADC1_SE5b);
}

void MagneticApp::DirectionControl()
{
	InitAll();
//			m_MagneticSenlx_filter=libutil::KalmanFilter(180000000.0f,1800000.071f,sen_lx_pred,1.0f);//618203.071f
//			m_MagneticSenly_filter=libutil::KalmanFilter(180000000.0f, 20000.603f,sen_ly_pred,1.0f);
//			m_MagneticSenrx_filter=libutil::KalmanFilter(180000000.0f, 793024.85413,sen_rx_pred,1.0f);
////			m_MagneticSenry_filter=libutil::KalmanFilter(180000000.0f, 20000.5649f,sen_ry_pred,1.0f);
//					m_MagneticSenlx_filter=libutil::KalmanFilter(0.0005f,5000.0f,adc_average(ADC0_SE10,ADC_16bit,100)*1.454545554,1.0f);//618203.071f
//					m_MagneticSenly_filter=libutil::KalmanFilter(0.0005f,5000.0f,adc_average(ADC1_SE4a,ADC_16bit,100),1.0f);
//					m_MagneticSenrx_filter=libutil::KalmanFilter(0.0005f,5000.0f,adc_average(ADC0_SE14,ADC_16bit,100)*1.5,1.0f);
//					m_MagneticSenry_filter=libutil::KalmanFilter(0.0005f,5000.0f,adc_average(ADC0_SE17,ADC_16bit),1.0f);


			LEFT_SENSOR_VALUE_X_filter = m_MagneticSenlx_filter.Filter(adc_average(ADC0_SE10,ADC_16bit,100)*1.454545554);
			LEFT_SENSOR_VALUE_Y_filter = m_MagneticSenly_filter.Filter(adc_average(ADC1_SE4a,ADC_16bit,100));
			RIGHT_SENSOR_VALUE_X_filter = m_MagneticSenrx_filter.Filter((adc_average(ADC0_SE14,ADC_16bit,100)*1.5));
			RIGHT_SENSOR_VALUE_Y_filter = m_MagneticSenry_filter.Filter(adc_average(ADC0_SE17,ADC_16bit,100));



			sen_lx_pred = LEFT_SENSOR_VALUE_X_filter;
			sen_ly_pred = LEFT_SENSOR_VALUE_Y_filter;
			sen_rx_pred = RIGHT_SENSOR_VALUE_X_filter;
			sen_ly_pred = RIGHT_SENSOR_VALUE_Y_filter;
			//if (RIGHT_SENSOR_VALUE_Y<8500)(RIGHT_SENSOR_VALUE_Y=RIGHT_SENSOR_VALUE_Y/8);
			//if (RIGHT_SENSOR_VALUE_Y <9500&&RIGHT_SENSOR_VALUE_Y >4000)(RIGHT_SENSOR_VALUE_Y=RIGHT_SENSOR_VALUE_Y/2.89);
			//if (RIGHT_SENSOR_VALUE_Y_filter >19500.0f)(RIGHT_SENSOR_VALUE_Y_filter=RIGHT_SENSOR_VALUE_Y_filter/2.625f);
			//if (LEFT_SENSOR_VALUE_Y_filter >19500.0f)(LEFT_SENSOR_VALUE_Y_filter=LEFT_SENSOR_VALUE_Y_filter/2.21052631579f);
			//LEFT_SENSOR_VALUE_X= (round((LEFT_SENSOR_VALUE_X)/150))*150;
			//RIGHT_SENSOR_VALUE_X= (round(RIGHT_SENSOR_VALUE_X/150))*150;
			LEFT_SENSOR_VALUE_X_filter=LEFT_SENSOR_VALUE_X_filter-10550;
			LEFT_SENSOR_VALUE_Y_filter=(LEFT_SENSOR_VALUE_Y_filter);//-18500)*2;
			RIGHT_SENSOR_VALUE_X_filter=RIGHT_SENSOR_VALUE_X_filter-7100;
			RIGHT_SENSOR_VALUE_Y_filter=(RIGHT_SENSOR_VALUE_Y_filter);//-16300)*2;
			//if( LEFT_SENSOR_VALUE_X_filter<10500)(LEFT_SENSOR_VALUE_X_filter=LEFT_SENSOR_VALUE_X_filter/2);
			//float LEFT_SENSOR_VALUE_X_filter_RAW= LEFT_SENSOR_VALUE_X_filter;
			if( LEFT_SENSOR_VALUE_X_filter<1)(LEFT_SENSOR_VALUE_X_filter=1);
			//float RIGHT_SENSOR_VALUE_X_filter_RAW= RIGHT_SENSOR_VALUE_X_filter;
			//RIGHT_SENSOR_VALUE_Y_filter=RIGHT_SENSOR_VALUE_Y_filter-3000;
			if(RIGHT_SENSOR_VALUE_X_filter<1)(RIGHT_SENSOR_VALUE_X_filter=1);
			//LEFT_SENSOR_VALUE_Y_filter=LEFT_SENSOR_VALUE_Y_filter-(15646.32);
			if( LEFT_SENSOR_VALUE_Y_filter<1)(LEFT_SENSOR_VALUE_Y_filter=1);
			//RIGHT_SENSOR_VALUE_Y_filter=RIGHT_SENSOR_VALUE_Y_filter-(15600);
			if(RIGHT_SENSOR_VALUE_Y_filter<1)(RIGHT_SENSOR_VALUE_Y_filter=1);
			LEFT_SENSOR_VALUE_X_filter_RAW =adc_average(ADC0_SE10,ADC_16bit,100)*1.454545554;
			LEFT_SENSOR_VALUE_Y_filter_RAW =adc_average(ADC1_SE4a,ADC_16bit,100);
			RIGHT_SENSOR_VALUE_X_filter_RAW = (adc_average(ADC0_SE14,ADC_16bit,100)*1.5);
			RIGHT_SENSOR_VALUE_Y_filter_RAW = adc_average(ADC0_SE17,ADC_16bit,100);
			LEFT_SENSOR_VALUE_X_filter_RAW=LEFT_SENSOR_VALUE_X_filter_RAW-7550;
			LEFT_SENSOR_VALUE_Y_filter_RAW=(LEFT_SENSOR_VALUE_Y_filter_RAW);//-18500)*2;
			RIGHT_SENSOR_VALUE_X_filter_RAW=RIGHT_SENSOR_VALUE_X_filter_RAW-7100;
			RIGHT_SENSOR_VALUE_Y_filter_RAW=(RIGHT_SENSOR_VALUE_Y_filter_RAW);//-16300)*2;
			if( LEFT_SENSOR_VALUE_X_filter_RAW<1)(LEFT_SENSOR_VALUE_X_filter_RAW=1);
			if(RIGHT_SENSOR_VALUE_X_filter_RAW<1)(RIGHT_SENSOR_VALUE_X_filter_RAW=1);
			if( LEFT_SENSOR_VALUE_Y_filter_RAW<1)(LEFT_SENSOR_VALUE_Y_filter_RAW=1);
			if(RIGHT_SENSOR_VALUE_Y_filter_RAW<1)(RIGHT_SENSOR_VALUE_Y_filter_RAW=1);
			float Y_Diff = RIGHT_SENSOR_VALUE_Y_filter-LEFT_SENSOR_VALUE_Y_filter;
			if (Y_Diff>7000||Y_Diff<-7000)(LEFT_SENSOR_VALUE_X_filter=2000,RIGHT_SENSOR_VALUE_X_filter=2000,LEFT_SENSOR_VALUE_X_filter_RAW=2000,RIGHT_SENSOR_VALUE_X_filter_RAW=2000);
			//if (Y_Diff>7000||Y_Diff<-7000)(RIGHT_SENSOR_VALUE_Y_filter_RAW=RIGHT_SENSOR_VALUE_Y_filter_RAW*2,RIGHT_SENSOR_VALUE_Y_filter=RIGHT_SENSOR_VALUE_Y_filter*2,LEFT_SENSOR_VALUE_Y_filter_RAW=LEFT_SENSOR_VALUE_Y_filter_RAW*2,LEFT_SENSOR_VALUE_Y_filter=LEFT_SENSOR_VALUE_Y_filter*2);
			if (Y_Diff<-7000)(RIGHT_SENSOR_VALUE_Y_filter_RAW=2000,RIGHT_SENSOR_VALUE_Y_filter=2000);
			if (Y_Diff>7000)(LEFT_SENSOR_VALUE_Y_filter_RAW=2000,LEFT_SENSOR_VALUE_Y_filter=2000);
			LEFT_FINAL=(((LEFT_SENSOR_VALUE_Y_filter)*(LEFT_SENSOR_VALUE_Y_filter))+LEFT_SENSOR_VALUE_X_filter*LEFT_SENSOR_VALUE_X_filter)/10000;
			LEFT_FINAL=sqrt(LEFT_FINAL)*100;
			RIGHT_FINAL=(((RIGHT_SENSOR_VALUE_Y_filter)*(RIGHT_SENSOR_VALUE_Y_filter))+RIGHT_SENSOR_VALUE_X_filter*RIGHT_SENSOR_VALUE_X_filter)/10000;
			RIGHT_FINAL=sqrt(RIGHT_FINAL)*100;//*1.28947368421;//-DELTA_VALUE_OFFSET;
			LEFT_FINAL_RAW=(((LEFT_SENSOR_VALUE_Y_filter_RAW)*(LEFT_SENSOR_VALUE_Y_filter_RAW))+LEFT_SENSOR_VALUE_X_filter_RAW*LEFT_SENSOR_VALUE_X_filter_RAW)/10000;
			LEFT_FINAL_RAW=sqrt(LEFT_FINAL_RAW)*100;
			RIGHT_FINAL_RAW=(((RIGHT_SENSOR_VALUE_Y_filter_RAW)*(RIGHT_SENSOR_VALUE_Y_filter_RAW))+RIGHT_SENSOR_VALUE_X_filter_RAW*RIGHT_SENSOR_VALUE_X_filter_RAW)/10000;
			RIGHT_FINAL_RAW=sqrt(RIGHT_FINAL_RAW)*100*1.28947368421;//-DELTA_VALUE_OFFSET;
			//RIGHT_SENSOR_VALUE_BACK=(round((RIGHT_SENSOR_VALUE_BACK)/250))*250;
			//theta= atan(MID_SENSOR_VALUE/(LEFT_SENSOR_VALUE-2120));
			//LEFT_SENSOR_VALUE=(LEFT_SENSOR_VALUE+2120)*cos(atan(MID_SENSOR_VALUE/(LEFT_SENSOR_VALUE+2120)));
			//RIGHT_SENSOR_VALUE=RIGHT_SENSOR_VALUE*cos(atan((RIGHT_SENSOR_VALUE)/MID_SENSOR_VALUE));
			//temp=(RIGHT_FINAL-LEFT_FINAL);
			temp_RAW=RIGHT_FINAL_RAW-LEFT_FINAL_RAW;
			DELTA_SENSOR_VALUE_RAW=20000*(temp_RAW)/(LEFT_FINAL_RAW + RIGHT_FINAL_RAW);
			if(LEFT_SENSOR_VALUE_X_filter<500&&LEFT_SENSOR_VALUE_Y_filter<500&&RIGHT_SENSOR_VALUE_X_filter<500&&RIGHT_SENSOR_VALUE_Y_filter<500)(temp_RAW=1,DELTA_SENSOR_VALUE_RAW=1);
			if(DELTA_SENSOR_VALUE_prev*DELTA_SENSOR_VALUE<0)(DELTA_SENSOR_VALUE=DELTA_SENSOR_VALUE_prev);
			int abc=20000*(RIGHT_FINAL-LEFT_FINAL)/(LEFT_FINAL + RIGHT_FINAL);
			DELTA_deter_spd=abc;
}

#define THR1		3000
#define THR2		3200
#define THR_turn    150
float turn_scale = 1.0f;

void MagneticApp::CaluServoPercentage()
{
	//const Timer::TimerInt prev_time;
	const Timer::TimerInt time = System::Time();
	const Timer::TimerInt time_diff = Timer::TimeDiff(time, prev_time);
	//m_car.LcdSetRow(4);
	//m_car.LcdPrintString(libutil::String::Format("timediff: %d\n", time_diff).c_str(), 0xFFFF);
	//0.3768/1000000;
	if(LEFT_SENSOR_VALUE_X_filter_RAW < 1000 || RIGHT_SENSOR_VALUE_X_filter_RAW < 1800){
		state_flag = 2;
		//turn_scale++;
	}
	else if (LEFT_FINAL_RAW >3000&& RIGHT_FINAL_RAW>3000&&temp_RAW<THR1&&temp_RAW>(-THR1)){
		pos_kp=0.22, pos_kd=0.95;
		prev_state_flag = state_flag = 1;
		m_MagneticSenlx_filter=libutil::KalmanFilter(0.05f,50.0f,adc_average(ADC0_SE10,ADC_16bit,100)*1.454545554,1.0f);//618203.071f
		m_MagneticSenly_filter=libutil::KalmanFilter(18000.0f,20000.0f,adc_average(ADC1_SE4a,ADC_16bit,100),1.0f);
		m_MagneticSenrx_filter=libutil::KalmanFilter(0.05f,50.0f,adc_average(ADC0_SE14,ADC_16bit,100)*1.5,1.0f);
		m_MagneticSenry_filter=libutil::KalmanFilter(18000.0f,20000.0f,adc_average(ADC0_SE17,ADC_16bit,100),1.0f);
		m_car.LcdSetRow(2);
		m_car.LcdPrintString(libutil::String::Format("STRAIGHT").c_str(), 0xFFFF);
		m_car.LcdClear(2);
	}
	/*else if((temp_RAW>(THR2-5000)&&temp_RAW<THR2)||(temp_RAW<(-THR2+5000)&&temp_RAW>-THR2)){
			pos_kp=0.85 ,pos_kd=0.45;
			prev_state_flag = state_flag = 0;
			m_car.LcdSetRow(2);
			m_car.LcdPrintString(libutil::String::Format("S_TURN").c_str(), 0xFFFF);
			m_car.LcdClear(2);
	}*/
	else if((temp_RAW>THR2||temp_RAW<-THR2)){
		pos_kp=0.65f,pos_kd=0.2f;
		prev_state_flag = state_flag = 0;
		m_car.LcdSetRow(2);
		m_car.LcdPrintString(libutil::String::Format("TURN").c_str(), 0xFFFF);
		m_car.LcdClear(2);
	}
	if(state_flag == 1 || (state_flag == 2 && prev_state_flag == 1)){
		turn_scale = 1.0f;
		DELTA_SENSOR_VALUE =20000*(RIGHT_FINAL-LEFT_FINAL)/(LEFT_FINAL + RIGHT_FINAL);
	}
	else if(state_flag == 0){
		turn_scale = 1.0f;
		DELTA_SENSOR_VALUE =DELTA_SENSOR_VALUE_RAW;
	}
	/*else if(state_flag == 2){
		//turn_scale+=0.0f;
		DELTA_SENSOR_VALUE=DELTA_prev_turn * turn_scale;
	}
	*/
	if (temp_RAW>16500||temp_RAW<-16500)(DELTA_prev_turn=DELTA_SENSOR_VALUE);
	if (temp_RAW<THR_turn&&temp_RAW>-THR_turn&&LEFT_FINAL <50&& RIGHT_FINAL<50)(DELTA_SENSOR_VALUE=DELTA_prev_turn);

	m_car.LcdSetRow(5);
	m_car.LcdPrintString(libutil::String::Format("%d\n",DELTA_SENSOR_VALUE).c_str(), 0xFFFF);
	//printf("Plot:%g,%g,%g,%g\r\n",LEFT_SENSOR_VALUE_X_filter,LEFT_SENSOR_VALUE_Y_filter,RIGHT_SENSOR_VALUE_X_filter,RIGHT_SENSOR_VALUE_Y_filter);
	//LOG_W("%d,%d",DELTA_SENSOR_VALUE,RIGHT_FINAL);
	//if (intrack==false)(DELTA_SENSOR_VALUE=0);
	slope =(DELTA_SENSOR_VALUE-DELTA_SENSOR_VALUE_prev);
	DELTA_SENSOR_VALUE_prev=DELTA_SENSOR_VALUE;
	//if(LEFT_SENSOR_VALUE_X_filter_RAW<1500.0f&&RIGHT_SENSOR_VALUE_X_filter_RAW<1500.0f)(intrack=0);
	//if (DELTA_SENSOR_VALUE>4000)(pos_kp=pos_kp*1.5);
	//if(DELTA_SENSOR_VALUE>0)(DELTA_SENSOR_VALUE=DELTA_SENSOR_VALUE*0.83333);
	//if (slope >200) (slope=200);
	//if (slope <-200) (slope=-200);
	//LOG_W("%d",err);
	DELTA_SENSOR_VALUE_ADJ =pos_kp*(float)(DELTA_SENSOR_VALUE)/82+ pos_kd * slope/(time_diff*80);
	//int off_ADJ=pos_kp*(float)(DELTA_VALUE_OFFSET)/82;
	//if (DELTA_SENSOR_VALUE_ADJ > 20) (DELTA_SENSOR_VALUE_ADJ=DELTA_SENSOR_VALUE_ADJ*1.4);
	//if(DELTA_SENSOR_VALUE_ADJ<-65) DELTA_SENSOR_VALUE_ADJ=DELTA_SENSOR_VALUE_ADJ-15;
	//if(DELTA_SENSOR_VALUE_ADJ>65) DELTA_SENSOR_VALUE_ADJ=DELTA_SENSOR_VALUE_ADJ+15;
	if(DELTA_SENSOR_VALUE_ADJ>(-10)&&DELTA_SENSOR_VALUE_ADJ<(10))
		(DELTA_SENSOR_VALUE=0);
	if (DELTA_SENSOR_VALUE_ADJ>120)
		(DELTA_SENSOR_VALUE_ADJ =120);
	if (DELTA_SENSOR_VALUE_ADJ <-120)
		(DELTA_SENSOR_VALUE_ADJ =-120);
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
	//m_car.LcdSetRow(3);
	//m_car.LcdPrintString(libutil::String::Format("slope: %d\n", slope).c_str(), 0xFFFF);
	prev_time = time;

}

void MagneticApp::SpdCtrlpass() //encoder + speed control pid
{
	//float spd_redu_ratio=10;
	//if (DELTA_SENSOR_VALUE_ADJ>50||DELTA_SENSOR_VALUE_ADJ<-50)(spd=2500);
	const Timer::TimerInt time = System::Time();
	const Timer::TimerInt time_diff = Timer::TimeDiff(time, prev_time);
	m_instance->m_car.UpdateEncoder();
	encodercountr = m_instance->m_car.GetEncoderCount(0);
	encodercountr=encodercountr*1.18181818182;
	encodercountl = m_instance->m_car.GetEncoderCount(1);
	//encodercountl=77*encodercountl/72;
	//if(DELTA_SENSOR_VALUE_ADJ < -70||DELTA_SENSOR_VALUE_ADJ >70)(spd=2200);
	//spd=2800;//while 7.76volt
	spd=550;
	//2if(DELTA_SENSOR_VALUE_ADJ >5)(spdr=spdr*2.1);
	//if(DELTA_SENSOR_VALUE_ADJ>60||DELTA_SENSOR_VALUE_ADJ<-60)
			//spd= spd*0.75;

	//if (motor_dir==1) (encodercountr=-1*encodercountr);
	//spdl_prop=spd_kp*((spd-encodercountr)/spd);
	//int spdr_prop=spd-encodercountr;
	// int spd_rt=spd*(285-DELTA_SENSOR_VALUE_ADJ)/285, spd_lt = spd*(285+DELTA_SENSOR_VALUE_ADJ)/285;
	 //int spd_rt=spd*1.14, spd_lt = spd*0.86;
	 /*if (DELTA_SENSOR_VALUE_ADJ>50)
    spdr=((spd_rt)*(1+(5.0*(spd_rt-encodercountr)/spd_rt))),spdl=(spd_lt*(1+(5.0*(spd_lt-encodercountl)/spd_lt)));//+time_diff/2*spd_ki*(spdl_err_prev+spdl_err);
	 else if (DELTA_SENSOR_VALUE_ADJ<-50)
		 spdr=(spd_rt*(1+(5.0*(spd_rt-encodercountr)/spd_rt))),spdl=(spd_lt*(1+(5.0*(spd_lt-encodercountl)/spd_lt)));
	 else
	spdr=(spd*(1+(5.0*(spd-encodercountr)/spd))),spdl=(spd*(1+(5.0*(spd-encodercountl)/spd)));
	*/
	//int spd_fs=spd*(1+17/(2*abs(DELTA_SENSOR_VALUE_ADJ)/2.14)),
	//int spd_sl = spd-spd*24/(2*abs(DELTA_SENSOR_VALUE_ADJ)/3);
	int slopel =spdl_err-spdl_err_prev,sloper=spdr_err-spdr_err_prev;
	/*if (DELTA_SENSOR_VALUE_ADJ>55)
	spdl_err=spd-encodercountl,spdr_err=spd_sl-encodercountr,spd_kp=1.1,
	spdr=(spd_sl*(1+(spd_kp*(spdr_err)/spd_sl)+spd_kd*sloper/(time_diff*80))),spdl=(spd*(1+(spd_kp*(spdl_err)/spd)+spd_kd*slopel/(time_diff*80)));
	else if (DELTA_SENSOR_VALUE_ADJ<-55)
	spdl_err=spd_sl-encodercountl,spdr_err=spd-encodercountr,spd_kp=1.1,
	spdr=(spd*(1+(spd_kp*(spdr_err)/spd)+spd_kd*sloper/(time_diff*80))),spdl=(spd_sl*(1+(spd_kp*(spdl_err)/spd_sl)+spd_kd*slopel/(time_diff*80)));
	else*/
	spdl_err=spd-encodercountl,spdr_err=spd-encodercountr;
	float left_prop = (spd_kp*(spdl_err)),right_prop=(spd_kp*(spdr_err));

	total_error_l += spdl_err;
	total_error_r += spdr_err;


	spdr = (float)right_prop + spd_kd*sloper*1000 + spd_ki * total_error_r;
	spdl = (float)left_prop + spd_kd*slopel*1000 + spd_ki * total_error_l;

	 //if (spdr<0)(motor_dir=1,spdr=abs(spdr));
   // else(motor_dir=0);
	//m_instance->m_car.UartSendStr(libutil::String::Format("spdl: %d, spdr: %d\n", spdl, spdr).c_str());
    //LOG_W("%d,%d",encodercountl,encodercountr)
    //printf("%d\n",encodercountl);
	m_instance->m_car.SetMotorRightDirection(0),m_instance->m_car.SetMotorRightDirection(0);

    if (spdr<0) spdr=0;
    if (spdl<0) spdl=0;
	if (encodercountr<100||encodercountl<100){
		//if(protected_count++ > 3){
			intrack= false;
		//}
	}else{
		protected_count = 0;
	}
	if ((temp_RAW<10&&temp_RAW>-10)&&(temp_RAW_prev<10&&temp_RAW_prev>-10)&&LEFT_SENSOR_VALUE_X_filter <10&& RIGHT_SENSOR_VALUE_X_filter<10)(intrack= false);
	//if (intrack == false) (spdr=0,spdl=0);
	//m_instance->m_car.SetMotorPowerLeft(libutil::Clamp<int>(-10000, 2500, 10000)),m_instance->m_car.SetMotorPowerRight((libutil::Clamp<int>(-10000, 2500, 10000)));
	m_instance->m_car.SetMotorPowerLeft((float)spdl);
	m_instance->m_car.SetMotorPowerRight((float) spdr);

	m_car.LcdSetRow(1);
	m_car.LcdPrintString(libutil::String::Format("%d\n",time_diff).c_str(), 0xFFFF);
	m_car.LcdSetRow(2);
	m_car.LcdPrintString(libutil::String::Format("encoderl: %d\n", encodercountl).c_str(), 0xFFFF);

	prev_time = time;
	spdr_err_prev=spdr_err;
	spdl_err_prev=spdl_err;
	encodercountr_prev=encodercountr;
	//spd_fs=0,spd_sl=0;
	//spd_sl=0;
	temp_RAW_prev=temp_RAW;
	//printf("%f,%f\n",spdl,spdr);
}
uint16 MagneticApp::CalcOffset(uint16 count_no)
{

		uint32 tmp = 0;
		    uint8  i;
		    for(i = 0; i < count_no; i++)
		    	DirectionControl();
		        tmp += temp_RAW;
		    tmp = tmp / count_no;
		    return (uint16)tmp;

}
int MagneticApp::DipSWget()
{
	int i,j,k,l;
	k=gpio_get(PTE12);
	j=gpio_get(PTE11);
	i=gpio_get(PTE10);
	l=gpio_get(PTE9);
	//int dip_sw_bin[4]={l,k,j,i};
	int tmp=l+i*2+j*4+k*8;
	return(tmp);
}

void MagneticApp::Run()
{
	System::Init();

	DELAY_MS(2000);
	//DELTA_VALUE_OFFSET=CalcOffset(40);
	InitAll();
	LEFT_SENSOR_VALUE_X = adc_average(ADC0_SE10,ADC_16bit,40);
	RIGHT_SENSOR_VALUE_X = adc_average(ADC0_SE14,ADC_16bit,40);
if (LEFT_SENSOR_VALUE_X>1500&&RIGHT_SENSOR_VALUE_X>1500) (intrack=true);
	__g_fwrite_handler = FwriteHandler;
	__g_hard_fault_handler = HardFaultHandler;

	/*SetIsr(PIT3_VECTORn, ServoFTMHandler);
	pit_init_ms(PIT3, 5);
	EnableIsr(PIT3_VECTORn);*/
	//DELAY_MS(1500);
//intrack==
	sys_mode=DipSWget();

	Timer::TimerInt prev_servo = 0, prev_spd = 0;
	while (sys_mode==1)
	{
		//InitAll();
		const Timer::TimerInt now = System::Time();
		if (Timer::TimeDiff(now, prev_servo) >= 5)
		{
			prev_servo = now;
			DirectionControl();
			CaluServoPercentage();
			m_car.SetTurning(DELTA_SENSOR_VALUE_ADJ);
			//printf("Plot:%d,%d%d,%d\r\n",a,b,c,d);
			//printf("%f,%f,%f,%f\n",LEFT_SENSOR_VALUE_X_filter,LEFT_SENSOR_VALUE_Y_filter,RIGHT_SENSOR_VALUE_X_filter,RIGHT_SENSOR_VALUE_Y_filter);
			//printf("%d,%d,%d,%d\n",adc_average(ADC0_SE10,ADC_16bit,40),adc_average(ADC1_SE4a,ADC_16bit,40),adc_average(ADC0_SE14,ADC_16bit,40),adc_average(ADC0_SE17,ADC_16bit,40));
			//printf("%d\n",LEFT_SENSOR_VALUE_X);
		}

		if (Timer::TimeDiff(now, prev_spd) >= 20)
		{
			prev_spd = now;
			SpdCtrlpass();
		}
		////printf("%d,%d\n",LEFT_FINAL,RIGHT_FINAL);
		//DELTA_SENSOR_VALUE_ADJ_prev=DELTA_SENSOR_VALUE_ADJ;
		//printf("Plot:%g,%g,%g,%g\r\n",LEFT_SENSOR_VALUE_X_filter,LEFT_SENSOR_VALUE_Y_filter,RIGHT_SENSOR_VALUE_X_filter,RIGHT_SENSOR_VALUE_Y_filter);
		//m_car.LcdSetRow(1);
		//m_car.LcdPrintString(libutil::String::Format("%f\n",DELTA_VALUE_OFFSET).c_str(), 0xFFFF);

		//m_car.LcdSetRow(6);
		//m_car.LcdPrintString(libutil::String::Format("left_spd:%d\n",spdl).c_str(), 0xFFFF);
		//m_car.LcdSetRow(7);
		//m_car.LcdPrintString(libutil::String::Format("right_spd:%d\n",spdr).c_str(), 0xFFFF);

		//m_instance->m_car.SetMotorPowerLeft(1800),m_instance->m_car.SetMotorPowerRight(1800);
	}
		while (sys_mode==2)
			{
				DirectionControl();
				CaluServoPercentage();
				m_car.SetTurning(DELTA_SENSOR_VALUE_ADJ);
				m_car.LcdSetRow(1);
				m_car.LcdPrintString(libutil::String::Format("%d\n",DELTA_SENSOR_VALUE).c_str(), 0xFFFF);
				m_car.LcdSetRow(8);
				m_car.LcdPrintString(libutil::String::Format("%d\n",DELTA_prev_turn).c_str(), 0xFFFF);
			}
		while(sys_mode==8)//y-direction sensors test
			{
				DirectionControl();
				CaluServoPercentage();
				m_car.SetTurning(DELTA_SENSOR_VALUE_ADJ);
				sys_mode=DipSWget();
				m_car.LcdSetRow(1);
				m_car.LcdPrintString(libutil::String::Format("%f\n",LEFT_SENSOR_VALUE_Y_filter_RAW).c_str(), 0xFFFF);
				m_car.LcdSetRow(3);
				m_car.LcdPrintString(libutil::String::Format("%f\n",RIGHT_SENSOR_VALUE_Y_filter_RAW).c_str(), 0xFFFF);

			}
		while(sys_mode==9)//x-direction sensors test
			{
				DirectionControl();
				CaluServoPercentage();
				m_car.SetTurning(DELTA_SENSOR_VALUE_ADJ);
				sys_mode=DipSWget();
				m_car.LcdSetRow(1);
				m_car.LcdPrintString(libutil::String::Format("%f\n",LEFT_SENSOR_VALUE_X_filter_RAW).c_str(), 0xFFFF);
				m_car.LcdSetRow(3);
				m_car.LcdPrintString(libutil::String::Format("%f\n",RIGHT_SENSOR_VALUE_X_filter_RAW).c_str(), 0xFFFF);
			}
		while(sys_mode==3)
			{
				SpdCtrlpass();
				sys_mode=DipSWget();
				m_car.LcdSetRow(1);
				m_car.LcdPrintString(libutil::String::Format("%d\n",spdl).c_str(), 0xFFFF);
				m_car.LcdSetRow(2);
				m_car.LcdPrintString(libutil::String::Format("%d\n",encodercountl).c_str(), 0xFFFF);
			}
		while(sys_mode==15)
		{

			sys_mode=DipSWget();
			m_car.LcdSetRow(1);
			m_car.LcdPrintString(libutil::String::Format("%d\n",sys_mode).c_str(), 0xFFFF);

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

void MagneticApp::HardFaultHandler(void)
{
	if (m_instance)
	{
		m_instance->m_car.StopMotor();
		for (int i = 0; i < 4; ++i)
		{
			m_instance->m_car.SwitchLed(i, true);
		}
		while (true)
		{}
	}
}

}

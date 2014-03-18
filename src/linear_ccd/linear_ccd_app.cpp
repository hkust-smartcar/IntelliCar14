/*
 * linear_ccd_app.cpp
 * Linear CCD App
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <syscall.h>

#include <cstdint>

#include <libsc/com/linear_ccd.h>
#include <libutil/clock.h>
#include <libutil/string.h>

#include "libutil/pid_controller.h"
#include "libutil/pid_controller.tcc"

#include "linear_ccd/car.h"

#include "linear_ccd/linear_ccd_app.h"

using libutil::Clock;

#define LED_FREQ 250
#define SERVO_FREQ 2
#define SPEED_CTRL_FREQ 100

#define SPEED_SP 300
#define SPEED_KP 2.666f
#define SPEED_KI 0.0f
#define SPEED_KD 8.0f

#define vaild_pixel 244

int32_t left_start_length  = 25;
int32_t right_start_length = 25;

int32_t ccd_mid_pos = 121;

int all_white_smaple_flag = 0;
int all_black_smaple_flag = 0;

int current_mid_error_pos=121;
int last_sample_error_pos=121;
int previous_mid_error_pos=121;

uint32_t current_dir_error=0;
uint32_t current_dir_arc_value_error=0;

int current_1st_left_edge=243;
int current_1st_right_edge=0;

int32_t current_edge_middle_distance=0;

int detect_left_flag=0;
int detect_right_flag=0;

namespace linear_ccd
{

LinearCcdApp *LinearCcdApp::m_instance = nullptr;

LinearCcdApp::SpeedState::SpeedState()
		: prev_run(0)
{}

LinearCcdApp::LinearCcdApp()
{
	m_instance = this;
}

LinearCcdApp::~LinearCcdApp()
{
	m_instance = nullptr;
}

void LinearCcdApp::Run()
{
	__g_fwrite_handler = FwriteHandler;
	libutil::Clock::Init();

	while (true)
	{
		ServoPass();
		SpeedControlPass();
		LedPass();
	}
}

void LinearCcdApp::LedPass()
{
	const Clock::ClockInt time = Clock::Time();
	if (Clock::TimeDiff(time, m_led_state.prev_run) >= LED_FREQ)
	{
		m_car.SwitchLed(0, m_led_state.flag);
		m_car.SwitchLed(1, !m_led_state.flag);
		m_led_state.flag ^= true;

		m_led_state.prev_run = time - (time % LED_FREQ);
	}
}

void LinearCcdApp::ServoPass()
{
	const Clock::ClockInt time = Clock::Time();
	if (Clock::TimeDiff(time, m_servo_state.prev_run) >= SERVO_FREQ)
	{
		const bool *ccd_data = m_car.SampleCcd();
		// TODO
		// ccd_print(ccd_data);

#ifdef DEBUG
		// Send CCD data through UART
		char str[libsc::LinearCcd::SENSOR_W];
		for (int i = 0; i < libsc::LinearCcd::SENSOR_W; ++i)
		{
			str[i] = ccd_data[i] ? ' ' : '#';
		}
		m_car.UartSendStr(str);
#endif

		m_servo_state.prev_run = time;
	}
}

void LinearCcdApp::Algorithm(const bool *ccd_data){

	  volatile int i;
	  detect_left_flag = 0;
	  detect_right_flag = 0;
	  current_1st_left_edge=243;
	  current_1st_right_edge=0;

	  for( i = last_sample_error_pos ; i > 0 ; i--){ // scan from last_sample_error_pos to left edge
	    if(ccd_data[i] == false){
	      current_1st_left_edge = i;
	      detect_left_flag = 1;
	      i = 1;
	    }
	  }

	  for( i = last_sample_error_pos ; i < vaild_pixel ; i++){  // scan from last_sample_error_pos to right edge
	    if(ccd_data[i] == false){
	      current_1st_right_edge = i;
	      detect_right_flag = 1;
	      i = 243;
	    }
	  }

	  /* ||||--------------------------------|||| */
	  if(detect_left_flag == 1 && detect_right_flag == 1){
	    current_mid_error_pos = (current_1st_left_edge + current_1st_right_edge) / 2;
	    //left_start_length = current_mid_error_pos - current_1st_left_edge;
	    //right_start_length = current_mid_error_pos + current_1st_right_edge;
	  }

	  /* ||||--------------------------------||||
	     |||||||||||||||------------------------
	     |||||||||||||||||||||||--------------- */
	  else if(detect_left_flag == 1 && detect_right_flag == 0){
	    current_mid_error_pos = current_1st_left_edge + right_start_length;

	    if( current_1st_left_edge == (vaild_pixel - 1)){
	      current_mid_error_pos = ccd_mid_pos;
	    }
	  }

	   /* ||||-------------------------------||||
	      --------------------------|||||||||||||
	      -----------------|||||||||||||||||||||| */
	  else if(detect_left_flag == 0 && detect_right_flag == 1){
	    current_mid_error_pos = current_1st_right_edge - left_start_length;

	    if(current_1st_right_edge == 0){
	      current_mid_error_pos = ccd_mid_pos;
	    }
	  }

	   /* ---------------------------------------- (no middle noise) Cross road*/
	  if(all_white_smaple_flag == 1){
	    //current_mid_error_pos = ccd_mid_pos+(encoder_turn_error*35/100); // John added
	    current_mid_error_pos = ccd_mid_pos;
	  }

	   /* |||||||||||||||||||||||||||||||||||||||| (all black) */
	  if(all_black_smaple_flag == 1){
	    current_mid_error_pos = ccd_mid_pos;
	  }

	  current_dir_error = (current_mid_error_pos - ccd_mid_pos);
	  //current_dir_arc_value_error = atan(current_dir_error*(atan_multiply_value))*1000;

	  //previous_mid_error_pos = current_mid_error_pos;
	  last_sample_error_pos = current_mid_error_pos;
	  current_edge_middle_distance = current_1st_right_edge - current_1st_left_edge;

}

void LinearCcdApp::ccd_scan_all_white_or_all_black_sample(const bool *ccd_data){
  int i;
  int white_counter=0;
  int black_counter=0;
  all_white_smaple_flag= 0;
  all_black_smaple_flag= 0;

  for( i = 0 ; i < vaild_pixel ; i++){
        if(ccd_data[i] == true){
          white_counter++;
        }else if(ccd_data[i] == false){
          black_counter++;
        }
  }

  if(white_counter == vaild_pixel){
    all_white_smaple_flag = 1;
  } else if (black_counter == vaild_pixel){
    all_black_smaple_flag = 1;
  }
}

void ccd_print(const bool *ccd_data){
      int i;
      for( i = 0 ; i < 256 ; i++){
    	if(ccd_data[i]==false){
        printf("W");
    	}
    	else if(ccd_data[i]==true){
        printf("o");
    	}
      }
      printf("\n");
}

void LinearCcdApp::SpeedControlPass()
{
	const Clock::ClockInt time = Clock::Time();
	if (Clock::TimeDiff(time, m_speed_state.prev_run) >= SPEED_CTRL_FREQ)
	{

		const uint16_t power = m_speed_state.pid.Calc(time,
				m_car.GetEncoderCount());
		//m_car.SetMotorPower(power);

#ifdef DEBUG
		// Send speed PID through UART
		m_car.UartSendStr(libutil::String::Format("%u", power).c_str());
#endif
		const uint16_t power = 0;
		m_car.SetMotorPower(power);

		m_speed_state.prev_run = time;
	}
}

int LinearCcdApp::FwriteHandler(int, char *ptr, int len)
{
	if (m_instance)
	{
		m_instance->m_car.UartSendBuffer((const uint8_t*)ptr, len);
	}
	return len;
}

}

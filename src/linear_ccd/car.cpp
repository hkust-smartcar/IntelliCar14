/*
 * car.cpp
 * Linear CCD car
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdint>

#include <libbase/k60/dac.h>
#include <libsc/k60/button.h>
#include <libsc/k60/encoder.h>
#include <libsc/k60/joystick.h>
#include <libsc/k60/led.h>
#include <libsc/k60/light_sensor.h>
#include <libsc/k60/linear_ccd.h>
#include <libsc/k60/motor.h>
#include <libsc/k60/servo.h>
#include <libsc/k60/switch.h>
#include <libsc/k60/uart_device.h>
#include <libutil/misc.h>

#include "linear_ccd/debug.h"
#include "linear_ccd/car.h"

#define SERVO_MID_DEGREE 815
#define SERVO_AMPLITUDE 290
#define SERVO_MAX_DEGREE (SERVO_MID_DEGREE + SERVO_AMPLITUDE)
#define SERVO_MIN_DEGREE (SERVO_MID_DEGREE - SERVO_AMPLITUDE)

using namespace libbase::k60;
using namespace libsc::k60;

namespace linear_ccd
{

namespace
{

Dac::Config GetDacConfig()
{
	Dac::Config dc;
	dc.module = Dac::Name::kDac0;
	dc.data[0] = 0;
	dc.data_size = 1;
	return dc;
}

}

Car::Car(const LightSensor::OnDetectListener &light_sensor_listener)
		: m_dac(GetDacConfig()),
		  m_buttons{Button(0), Button(1)
#ifndef LINEAR_CCD_2014
				  , Button(2), Button(3)
#endif
				  },
		  m_encoder(0), m_joystick(0), m_lcd(true), m_lcd_console(&m_lcd),
		  m_leds{Led(0), Led(1), Led(2), Led(3)},
		  m_light_sensors{LightSensor(0, light_sensor_listener),
			  	  LightSensor(1, light_sensor_listener)},
		  m_ccds{LinearCcd(0), LinearCcd(1)}, m_motor(0, false), m_buzzer(0),
		  m_switches{Switch(0), Switch(1), Switch(2), Switch(3), Switch(4)},
		  m_servo(0), m_bt(0, libbase::k60::Uart::Config::BaudRate::k115200)
{
	SetMotorPower(0);
	m_servo.SetDegree(SERVO_MID_DEGREE);
	m_lcd.Clear(libutil::GetRgb565(0x33, 0xB5, 0xE5));
}

Car::Car()
		: Car(nullptr)
{}

void Car::UartEnableRx()
{
	m_bt.EnableRx();
}

void Car::SetMotorDirection(const bool is_forward)
{
	m_motor.SetClockwise(!is_forward);
}

void Car::SetMotorPower(const int16_t power)
{
	const Uint power_ = libutil::Clamp<Uint>(0, abs(power), 1000);
	SetMotorDirection((power >= 0));
	m_motor.SetPower(power_);
}

void Car::SetTurning(const int16_t percentage)
{
	// Servo's rotation dir is opposite to our wheels
	const int percentage_ = libutil::Clamp<int>(-100, -percentage, 100);
	const int degree = SERVO_MID_DEGREE + (percentage_ * SERVO_AMPLITUDE / 100);
	m_servo.SetDegree(degree);
}

bool Car::IsMotorForward() const
{
	return !m_motor.IsClockwise();
}

int16_t Car::GetTurning() const
{
	return (m_servo.GetDegree() - SERVO_MID_DEGREE) * -100 / SERVO_AMPLITUDE;
}

}

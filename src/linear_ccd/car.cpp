/*
 * car.cpp
 * Linear CCD car
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdint>

#include "linear_ccd/debug.h"

#include <libsc/k60/button.h>
#include <libsc/com/encoder.h>
#include <libsc/com/gyroscope.h>
#include <libsc/k60/joystick.h>
#include <libsc/k60/led.h>
#include <libsc/k60/light_sensor.h>
#include <libsc/k60/linear_ccd.h>
#include <libsc/com/motor.h>
#include <libsc/com/servo.h>
#include <libsc/k60/switch.h>
#include <libsc/k60/uart_device.h>
#include <libutil/misc.h>

#include "linear_ccd/car.h"

//#define SERVO_MID_DEGREE 86
//#define SERVO_AMPLITUDE 25
#define SERVO_MID_DEGREE 86
#define SERVO_AMPLITUDE 21
#define SERVO_MAX_DEGREE (SERVO_MID_DEGREE + SERVO_AMPLITUDE)
#define SERVO_MIN_DEGREE (SERVO_MID_DEGREE - SERVO_AMPLITUDE)

using namespace libsc;
using namespace libsc::k60;

namespace linear_ccd
{

Car::Car()
		: m_buttons{Button(0), Button(1)
#ifndef LINEAR_CCD_2014
				  , Button(2), Button(3)
#endif
				  },
		  m_encoder(0), m_joystick(0), m_lcd(true), m_lcd_console(&m_lcd),
		  m_leds{Led(0), Led(1), Led(2), Led(3)},
		  m_light_sensors{LightSensor(0), LightSensor(1)},
		  m_ccds{LinearCcd(0), LinearCcd(1)}, m_motor(0), m_buzzer(0),
		  m_switches{Switch(0), Switch(1), Switch(2), Switch(3), Switch(4)},
		  m_servo(0), m_bt(0, libbase::k60::UartConfig::BaudRate::BR_115200)
{
	SetMotorPower(0);
	m_servo.SetDegree(SERVO_MID_DEGREE);
	m_bt.StartReceive();
	m_lcd.Clear(libutil::GetRgb565(0x33, 0xB5, 0xE5));
}

void Car::SetMotorDirection(const bool is_forward)
{
	m_motor.SetClockwise(!is_forward);
}

void Car::SetMotorPower(const int16_t power)
{
	const uint16_t _power = libutil::Clamp<uint16_t>(0, abs(power), 10000);
	SetMotorDirection((power >= 0));
	m_motor.SetPower(_power);
}

void Car::AddMotorPowerTil(const uint16_t factor, const uint16_t max)
{
	const uint16_t curr_power = GetMotorPower();
	if (curr_power < max)
	{
		SetMotorPower(libutil::Clamp<uint16_t>(0, curr_power + factor, max));
	}
}

void Car::DropMotorPower(const uint16_t factor)
{
	m_motor.AddPower(-factor);
}

void Car::DropMotorPowerTil(const uint16_t factor, const uint16_t min)
{
	const uint16_t curr_power = GetMotorPower();
	if (curr_power > min)
	{
		SetMotorPower(libutil::Clamp<uint16_t>(min, curr_power - factor, 1000));
	}
}

void Car::SetTurning(const int16_t percentage)
{
	// Servo's rotation dir is opposite to our wheels
	const int16_t _percentage = libutil::Clamp<int16_t>(-100, -percentage, 100);
	m_servo.SetDegree(SERVO_MID_DEGREE + (_percentage * SERVO_AMPLITUDE / 100));
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

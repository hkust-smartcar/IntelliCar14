/*
 * car.cpp
 * Magnetic car
 *
 * Author:
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdint>

#include <libbase/k60/misc_utils.h>
#include <libbase/k60/uart.h>
#include <libsc/k60/futaba_s3010.h>
#include <libsc/k60/led.h>
#include <libsc/k60/motor.h>
#include <libsc/k60/uart_device.h>
#include <libutil/misc.h>
#include <libsc/com/encoder.h>

#include "magnetic/car.h"

#define SERVO_MID_DEGREE 830
#define SERVO_AMPLITUDE 500
#define SERVO_MAX_DEGREE (SERVO_MID_DEGREE + SERVO_AMPLITUDE)
#define SERVO_MIN_DEGREE (SERVO_MID_DEGREE - SERVO_AMPLITUDE)

using namespace libsc;
using namespace libsc::k60;

namespace magnetic
{

Car::Car()
		:m_encoder{Encoder(0), Encoder(1)},
		 m_servo(0), m_lcd(true), m_lcd_console(&m_lcd), m_leds{Led(0), Led(1), Led(2), Led(3)},
		 m_motor{Motor(0), Motor(1)}, m_uart(3, libbase::k60::Uart::Config::BaudRate::BR_115200)

{
			  SetMotorPowerLeft(0);
			  SetMotorPowerRight(0);
	SetMotorDirection(true);
	m_servo.SetDegree(SERVO_MID_DEGREE);
	m_uart.EnableRx();
}

void Car::SetMotorDirection(const bool is_forward)
{
	m_motor[0].SetClockwise(is_forward);
	m_motor[1].SetClockwise(!is_forward);
}

void Car::SetMotorRightDirection(const bool is_forward)
{
	m_motor[1].SetClockwise(!is_forward);
}
void Car::SetMotorLeftDirection(const bool is_forward)
{
m_motor[0].SetClockwise(is_forward);
}

void Car::SetTurning(const int16_t percentage)
{
	const int16_t _percentage = libutil::Clamp<int16_t>(-150, percentage, 150);
	m_servo.SetDegree(SERVO_MID_DEGREE + (_percentage * SERVO_AMPLITUDE / 150));
}

bool Car::IsMotorForward() const
{
	return !m_motor[0].IsClockwise();
}

int16_t Car::GetTurning() const
{
	return (m_servo.GetDegree() - SERVO_MID_DEGREE) * -150 / SERVO_AMPLITUDE;
}

}

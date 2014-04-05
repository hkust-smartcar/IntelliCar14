/*
 * car.cpp
 * Magnetic car
 *
 * Author:
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdint>

#include <libsc/com/led.h>
#include <libsc/com/uart_device.h>
#include <libutil/misc.h>

#include "magnetic/car.h"

#define SERVO_MID_DEGREE 90
#define SERVO_AMPLITUDE 56
#define SERVO_MAX_DEGREE (SERVO_MID_DEGREE + SERVO_AMPLITUDE)
#define SERVO_MIN_DEGREE (SERVO_MID_DEGREE - SERVO_AMPLITUDE)

using namespace libsc;

namespace magnetic
{

Car::Car()
		: m_servo(0), m_leds{Led(0), Led(1), Led(2), Led(3)},
		  m_motor{Motor(0), Motor(1)}, m_uart(3, 115200)
{
	SetMotorPower(0);
	SetMotorDirection(true);
	m_servo.SetDegree(SERVO_MID_DEGREE);
	m_uart.StartReceive();
}

void Car::SetMotorDirection(const bool is_forward)
{
	m_motor[0].SetClockwise(!is_forward);
	m_motor[1].SetClockwise(!is_forward);
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
	m_motor[0].AddPower(-factor);
	m_motor[1].AddPower(-factor);
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
	const int16_t _percentage = libutil::Clamp<int16_t>(-100, percentage, 100);
	m_servo.SetDegree(SERVO_MID_DEGREE + (_percentage * SERVO_AMPLITUDE / 100));
}

bool Car::IsMotorForward() const
{
	return !m_motor[0].IsClockwise();
}

uint8_t Car::GetRightPercentge() const
{
	return (m_servo.GetDegree() - SERVO_MIN_DEGREE) * 100 / (SERVO_AMPLITUDE * 2);
}

}

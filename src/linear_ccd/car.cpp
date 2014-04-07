/*
 * car.cpp
 * Linear CCD car
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdint>

#include <libsc/com/bluetooth.h>
#include <libsc/com/encoder.h>
#include <libsc/com/led.h>
#include <libsc/com/linear_ccd.h>
#include <libsc/com/motor.h>
#include <libsc/com/servo.h>
#include <libutil/misc.h>

#include "linear_ccd/car.h"

#define SERVO_MID_DEGREE 90
#define SERVO_AMPLITUDE 56
#define SERVO_MAX_DEGREE (SERVO_MID_DEGREE + SERVO_AMPLITUDE)
#define SERVO_MIN_DEGREE (SERVO_MID_DEGREE - SERVO_AMPLITUDE)

using namespace libsc;

namespace linear_ccd
{

Car::Car()
		: m_encoder(0), m_leds{Led(0), Led(1), Led(2), Led(3)}, m_motor(0),
		  m_servo(0)
{
	SetMotorPower(0);
	m_servo.SetDegree(SERVO_MID_DEGREE);
	m_bt.StartReceive();
}

void Car::SetMotorDirection(const bool is_forward)
{
	m_motor.SetClockwise(is_forward);
}

void Car::SetMotorPower(const int16_t power)
{
	const uint16_t _power = libutil::Clamp<uint16_t>(0, abs(power), 10000);
	SetMotorDirection((power >= 0));
	m_motor.SetPower(power);
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
	const int16_t _percentage = libutil::Clamp<int16_t>(-100, percentage, 100);
	m_servo.SetDegree(SERVO_MID_DEGREE + (_percentage * SERVO_AMPLITUDE / 100));
}

bool Car::IsMotorForward() const
{
	return !m_motor.IsClockwise();
}

uint8_t Car::GetRightPercentge() const
{
	return (m_servo.GetDegree() - SERVO_MIN_DEGREE) * 100 / (SERVO_AMPLITUDE * 2);
}

}

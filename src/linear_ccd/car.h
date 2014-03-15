/*
 * car.h
 * Linear CCD car
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_CAR_H_
#define LINEAR_CCD_CAR_H_

#include <libsc/com/bluetooth.h>
#include <libsc/com/encoder.h>
#include <libsc/com/led.h>
#include <libsc/com/linear_ccd.h>
#include <libsc/com/motor.h>
#include <libsc/com/servo.h>

namespace linear_ccd
{

class Car
{
public:
	Car();


	/**
	 * Start the wheel motor
	 *
	 * @param is_forward Is going forward or not
	 * @param power Power scale in [0, 1000]
	 */
	void StartMotor(const bool is_forward, const uint16_t power)
	{
		SetMotorDirection(is_forward);
		SetMotorPower(power);
	}

	void StopMotor()
	{
		SetMotorPower(0);
	}

	void SetMotorDirection(const bool is_forward);
	void SetMotorPower(const uint16_t power)
	{
		m_motor.SetPower(power);
	}

	void AddMotorPower(const uint16_t factor)
	{
		m_motor.AddPower(factor);
	}

	void AddMotorPowerTil(const uint16_t factor, const uint16_t max);
	void DropMotorPower(const uint16_t factor);
	void DropMotorPowerTil(const uint16_t factor, const uint16_t min);

	/**
	 * Turn the robot to the left
	 *
	 * @param percentage Specifying how aggressively should the car turn,
	 * in [0, 100], where passing 0 basically means going straight
	 */
	void TurnLeft(const uint8_t percentage);
	void TurnRight(const uint8_t percentage);
	/**
	 * Combining TurnLeft() and TurnRight() into one
	 *
	 * @param percentage 0 means turning left, and 70 means slightly turning
	 * right
	 * @return
	 */
	void SetRightPercentage(const uint8_t percentage);
	void GoStraight()
	{
		TurnLeft(0);
	}

	/**
	 * Switch on/off the LEDs
	 *
	 * @param obj
	 * @param id The id of the LED, [0, 3]
	 * @param flag
	 */
	void SwitchLed(const uint8_t id, const bool flag)
	{
		m_leds[id].SetEnable(flag);
	}

	const bool* SampleCcd()
	{
		return m_ccd.SampleData();
	}

	void UartSendStr(const char *str)
	{
		m_bt.SendStr(str);
	}

	void UartSendBuffer(const uint8_t *buf, const uint32_t len)
	{
		m_bt.SendBuffer(buf, len);
	}

	bool UartPeekChar(char *out_ch)
	{
		return m_bt.PeekChar(out_ch);
	}

	bool IsMotorForward() const;
	bool IsMotorStop() const
	{
		return !m_motor.GetPower();
	}

	uint16_t GetMotorPower() const
	{
		return m_motor.GetPower();
	}

	uint32_t GetEncoderCount()
	{
		return m_encoder.GetCount();
	}

	uint8_t GetRightPercentge() const;

private:
	libsc::Bluetooth m_bt;
	libsc::Encoder m_encoder;
	libsc::Led m_leds[4];
	libsc::LinearCcd m_ccd;
	libsc::Motor m_motor;
	libsc::Servo m_servo;
};

}

#endif /* LINEAR_CCD_CAR_H_ */
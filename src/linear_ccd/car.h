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
#include <libsc/com/trs_d05.h>

namespace linear_ccd
{

class Car
{
public:
	Car();

	/**
	 * Start the wheel motor
	 *
	 * @param power
	 * @see SetMotorPower()
	 */
	void StartMotor(const int16_t power)
	{
		SetMotorPower(power);
	}

	void StopMotor()
	{
		SetMotorPower(0);
	}

	/**
	 * Set the power of the motor, a negative power will drive the car backwards
	 *
	 * @param power Power scale in [-10000, 10000]
	 */
	void SetMotorPower(const int16_t power);

	void AddMotorPower(const uint16_t factor)
	{
		m_motor.AddPower(factor);
	}

	void AddMotorPowerTil(const uint16_t factor, const uint16_t max);
	void DropMotorPower(const uint16_t factor);
	void DropMotorPowerTil(const uint16_t factor, const uint16_t min);

	/**
	 * Set the turning percentage, negative input means turning left
	 *
	 * @param percentage Specifying how aggressively should the car turn,
	 * in [-100, 100], where passing 0 basically means going straight
	 */
	void SetTurning(const int16_t percentage);

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
	void SetMotorDirection(const bool is_forward);

	libsc::Bluetooth m_bt;
	libsc::Encoder m_encoder;
	libsc::Led m_leds[4];
	libsc::LinearCcd m_ccd;
	libsc::Motor m_motor;
	libsc::TrsD05 m_servo;
};

}

#endif /* LINEAR_CCD_CAR_H_ */

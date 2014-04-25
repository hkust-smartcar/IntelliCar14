/*
 * car.h
 * Magnetic car
 *
 * Author:
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef MAGNETIC_CAR_H_
#define MAGNETIC_CAR_H_

#include <libsc/com/futaba_s3010.h>
#include <libsc/com/led.h>
#include <libsc/com/motor.h>
#include <libsc/com/uart_device.h>
#include <libsc/com/encoder.h>

namespace magnetic
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
	void StartMotor(const bool is_forward, const uint16_t powerl, const uint16_t powerr)
	{
		SetMotorDirection(is_forward);
		SetMotorPowerLeft(powerl);
		SetMotorPowerRight(powerr);
	}

	void StopMotor()
	{
		SetMotorPowerLeft(0);
		SetMotorPowerRight(0);
	}

	void SetMotorDirection(const bool is_forward);
	void SetMotorPowerLeft(const uint16_t power)
	{
		m_motor[0].SetPower(power);
	}

	void SetMotorPowerRight(const uint16_t power)
	{
		m_motor[1].SetPower(power);
	}

	void AddMotorPower(const uint16_t factor)
	{
		m_motor[0].AddPower(factor);
		m_motor[1].AddPower(factor);
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

	void UpdateEncoder(const uint8_t id)
	{
		m_encoder[id].Update();
	}

	void UartSendStr(const char *str)
	{
		m_uart.SendStr(str);
	}

	void UartSendBuffer(const uint8_t *buf, const uint32_t len)
	{
		m_uart.SendBuffer(buf, len);
	}

	bool UartPeekChar(char *out_ch)
	{
		return m_uart.PeekChar(out_ch);
	}

	bool IsMotorForward() const;
	bool IsMotorStop() const
	{
		return !m_motor[0].GetPower();
	}

	uint16_t GetMotorPower() const
	{
		return m_motor[0].GetPower();
	}

	uint8_t GetRightPercentge() const;

	int16_t GetEncoderCount(const uint8_t id)
	{
		return m_encoder[id].GetCount();
	}

private:
	libsc::Encoder m_encoder[2];
	libsc::FutabaS3010 m_servo;
	libsc::Led m_leds[4];
	libsc::Motor m_motor[2];
	libsc::UartDevice m_uart;

};

}

#endif /* MAGNETIC_CAR_H_ */

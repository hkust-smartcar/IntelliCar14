/*
 * car.h
 * Linear CCD car
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_CAR_H_
#define LINEAR_CCD_CAR_H_

#include <bitset>

#include <libsc/com/bluetooth.h>
#include <libsc/com/button.h>
#include <libsc/com/encoder.h>
#include <libsc/com/gyroscope.h>
#include <libsc/com/lcd.h>
#include <libsc/com/lcd_console.h>
#include <libsc/com/led.h>
#include <libsc/com/light_sensor.h>
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

	void StartCcdSample()
	{
		m_ccd.StartSample();
	}

	bool CcdSampleProcess()
	{
		return m_ccd.SampleProcess();
	}

	const std::bitset<libsc::LinearCcd::SENSOR_W>& GetCcdSample() const
	{
		return m_ccd.GetData();
	}

	bool IsCcdReady() const
	{
		return m_ccd.IsImageReady();
	}

	void UpdateEncoder()
	{
		m_encoder.Update();
	}

	void UpdateGyro()
	{
		m_gyro.Update();
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

	void LcdDrawGrayscalePixelBuffer(const uint8_t x, const uint8_t y,
			const uint8_t w, const uint8_t h, const uint8_t *pixel)
	{
		m_lcd.DrawGrayscalePixelBuffer(x, y, w, h, pixel);
	}

	void LcdPrintString(const char *str, const uint16_t color)
	{
		m_lcd_console.PrintString(str, color);
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

	int16_t GetEncoderCount()
	{
		return m_encoder.GetCount();
	}

	int16_t GetTurning() const;

	/**
	 * Return the state of all 4 buttons packed in 1 byte. If it's currently
	 * down, the value will be 1, 0 otherwise
	 *
	 * @return
	 */
	uint8_t GetButtonState() const
	{
		uint8_t state = 0;
		for (int i = 0; i < 4; ++i)
		{
			state |= ((m_buttons[i].IsDown() ? 1 : 0) << i);
		}
		return state;
	}

	bool IsLightSensorDetected(const uint8_t id)
	{
		return m_light_sensors[id].IsDetected();
	}

	float GetGyroAngle() const
	{
		return m_gyro.GetAverageAngle();
	}

	int16_t GetEncoderCount() const
	{
		return m_encoder.GetCount();
	}

private:
	void SetMotorDirection(const bool is_forward);

	libsc::Bluetooth m_bt;
	libsc::Button m_buttons[4];
	libsc::Encoder m_encoder;
	libsc::Gyroscope m_gyro;
	libsc::Lcd m_lcd;
	libsc::LcdConsole m_lcd_console;
	libsc::Led m_leds[4];
	libsc::LightSensor m_light_sensors[2];
	libsc::LinearCcd m_ccd;
	libsc::Motor m_motor;
	libsc::TrsD05 m_servo;
};

}

#endif /* LINEAR_CCD_CAR_H_ */

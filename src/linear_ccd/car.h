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
#include <libsc/com/joystick.h>
#include <libsc/com/lcd.h>
#include <libsc/com/lcd_console.h>
#include <libsc/com/led.h>
#include <libsc/com/light_sensor.h>
#include <libsc/com/linear_ccd.h>
#include <libsc/com/motor.h>
#include <libsc/com/switch.h>
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

	void StartCcdSample(const uint8_t id)
	{
		m_ccds[id].StartSample();
	}

	bool CcdSampleProcess(const uint8_t id)
	{
		return m_ccds[id].SampleProcess();
	}

	const std::bitset<libsc::LinearCcd::SENSOR_W>& GetCcdSample(const uint8_t id) const
	{
		return m_ccds[id].GetData();
	}

	bool IsCcdReady(const uint8_t id) const
	{
		return m_ccds[id].IsImageReady();
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

	void LcdClear(const uint16_t)
	{
		m_lcd_console.Clear(false);
		// Skip clearing the screen as it's too slow
		//m_lcd.Clear(color);
	}

	void LcdSetRow(const uint8_t row)
	{
		m_lcd_console.SetCursorRow(row);
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
	 * Return the state of all buttons. The bit is set if it's currently down
	 *
	 * @return
	 */
#ifdef LINEAR_CCD_2014
	std::bitset<2> GetButtonState() const
	{
		std::bitset<2> state = 0;
		for (int i = 0; i < 2; ++i)
		{
			state[i] = m_buttons[i].IsDown();
		}
		return state;
	}

#else
	std::bitset<4> GetButtonState() const
	{
		std::bitset<4> state = 0;
		for (int i = 0; i < 4; ++i)
		{
			state[i] = m_buttons[i].IsDown();
		}
		return state;
	}

#endif

	/**
	 * Return the state of all switches. The bit is set if it's currently in
	 * the ON state
	 *
	 * @return
	 */
	std::bitset<5> GetSwitchState() const
	{
		std::bitset<5> state = 0;
		for (int i = 0; i < 5; ++i)
		{
			state[i] = m_switches[i].IsOn();
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

	libsc::Joystick::State GetJoystickState() const
	{
		return m_joystick.GetState();
	}

	int16_t GetEncoderCount() const
	{
		return m_encoder.GetCount();
	}

private:
	void SetMotorDirection(const bool is_forward);

#ifdef LINEAR_CCD_2014
	libsc::Button m_buttons[2];
#else
	libsc::Button m_buttons[4];
#endif
	libsc::Encoder m_encoder;
	libsc::Gyroscope m_gyro;
	libsc::Joystick m_joystick;
	libsc::Lcd m_lcd;
	libsc::LcdConsole m_lcd_console;
	libsc::Led m_leds[4];
	libsc::LightSensor m_light_sensors[2];
	libsc::LinearCcd m_ccds[1];
	libsc::Motor m_motor;
	libsc::Switch m_switches[5];
	libsc::TrsD05 m_servo;
	libsc::UartDevice m_bt;
};

}

#endif /* LINEAR_CCD_CAR_H_ */

/*
 * car.h
 * Linear CCD car
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <bitset>
#include <memory>
#include <string>
#include <utility>

#include <libbase/k60/dac.h>
#include <libsc/k60/button.h>
#include <libsc/k60/encoder.h>
#include <libsc/k60/joystick.h>
#include <libsc/k60/lcd_console.h>
#include <libsc/k60/led.h>
#include <libsc/k60/light_sensor.h>
#include <libsc/k60/linear_ccd.h>
#include <libsc/k60/dir_motor.h>
#include <libsc/k60/simple_buzzer.h>
#include <libsc/k60/st7735r.h>
#include <libsc/k60/switch.h>
#include <libsc/k60/trs_d05.h>
#include <libsc/k60/uart_device.h>
#include <libutil/remote_var_manager.h>

#include "linear_ccd/beep_manager.h"

namespace linear_ccd
{

class Car
{
public:
	explicit Car(const libsc::k60::LightSensor::OnDetectListener &light_sensor_listener);
	Car();

	/**
	 * Set the power of the motor, a negative power will drive the car backwards
	 *
	 * @param power Power scale in [-1000, 1000]
	 */
	void SetMotorPower(const int16_t power);

	void StopMotor()
	{
		SetMotorPower(0);
	}

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

	void SetCcdDacThreshold(const uint16_t val)
	{
		m_dac.SetData(val);
	}

	void StartCcdSample()
	{
		m_ccds[0].StartSample();
		//m_ccds[1].StartSample();
	}

	void StartCcdSample(const uint8_t id)
	{
		m_ccds[id].StartSample();
	}

	bool CcdSampleProcess()
	{
		return m_ccds[0].SampleProcess() /*&& m_ccds[1].SampleProcess()*/;
	}

	bool CcdSampleProcess(const uint8_t id)
	{
		return m_ccds[id].SampleProcess();
	}

	const std::bitset<libsc::k60::LinearCcd::kSensorW>& GetCcdSample(
			const uint8_t id) const
	{
		return m_ccds[id].GetData();
	}

	bool IsCcdReady() const
	{
		return m_ccds[0].IsImageReady() /*&& m_ccds[1].IsImageReady()*/;
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
		//m_gyro.Update();
	}

	void UartSendStr(const char *str)
	{
		m_bt.SendStr(str);
	}

	void UartSendStrLiteral(const char *str)
	{
		m_bt.SendStrLiteral(str);
	}

	void UartSendStr(std::string &&str)
	{
		m_bt.SendStr(std::move(str));
	}

	void UartSendBuffer(const Byte *buf, const size_t len)
	{
		m_bt.SendBuffer(buf, len);
	}

	bool UartPeekChar(char *out_ch)
	{
		return m_bt.PeekChar(out_ch);
	}

	void UartEnableRx();

	void SetUartLoopMode(const bool flag)
	{
		m_bt.SetLoopMode(flag);
	}

	void LcdDrawGrayscalePixelBuffer(const uint8_t x, const uint8_t y,
			const uint8_t w, const uint8_t h, const uint8_t *pixel)
	{
		m_lcd.DrawGrayscalePixelBuffer(x, y, w, h, pixel);
	}

	void LcdDrawPixel(const uint8_t x, const uint8_t y, const uint16_t pixel)
	{
		m_lcd.DrawPixel(x, y, pixel);
	}

	void LcdPrintString(const char *str, const uint16_t color)
	{
		m_lcd_console.PrintString(str, color);
	}

	void LcdPrintString(const char *str, const uint16_t color,
			const uint16_t bg_color)
	{
		m_lcd_console.PrintString(str, color, bg_color);
	}

	void LcdClear()
	{
		m_lcd_console.Clear(false);
	}

	void LcdClear(const uint16_t bg_color)
	{
		m_lcd_console.Clear(false);
		m_lcd.Clear(bg_color);
	}

	void LcdSetRow(const uint8_t row)
	{
		m_lcd_console.SetCursorRow(row);
	}

	BeepManager* GetBeepManager()
	{
		return BeepManager::GetInstance(&m_buzzer);
	}

	void SetBuzzerBeep(const bool is_beep)
	{
		m_buzzer.SetBeep(is_beep);
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
		return -(m_encoder.GetCount() >> 2);
	}

	int16_t GetTurning() const;

	void EnableRemoteVar(const size_t size);

	libutil::RemoteVarManager* GetRemoteVarManager()
	{
		return m_remote_var_manager.get();
	}

	/**
	 * Return the state of all buttons. The bit is set if it's currently down
	 *
	 * @return
	 */
#ifdef K60_2014_CCD
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
		//return m_gyro.GetAngle()[0];
		return 0;
	}

	libsc::k60::Joystick::State GetJoystickState() const
	{
		return m_joystick.GetState();
	}

	int16_t GetEncoderCount() const
	{
		return m_encoder.GetCount();
	}

private:
	void SetMotorDirection(const bool is_forward);

	libbase::k60::Dac m_dac;

#ifdef K60_2014_CCD
	libsc::k60::Button m_buttons[2];
#else
	libsc::k60::Button m_buttons[4];
#endif
	libsc::k60::Encoder m_encoder;
	libsc::k60::Joystick m_joystick;
	libsc::k60::St7735r m_lcd;
	libsc::k60::LcdConsole m_lcd_console;
	libsc::k60::Led m_leds[4];
	libsc::k60::LightSensor m_light_sensors[2];
	libsc::k60::LinearCcd m_ccds[2];
	libsc::k60::DirMotor m_motor;
	libsc::k60::SimpleBuzzer m_buzzer;
	libsc::k60::Switch m_switches[5];
	libsc::k60::TrsD05 m_servo;
	libsc::k60::UartDevice m_bt;

	std::unique_ptr<libutil::RemoteVarManager> m_remote_var_manager;
};

}

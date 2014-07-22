/*
 * config.h
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdint>

namespace linear_ccd
{

class Config
{
public:
	enum LcdScreenState
	{
		CCD_PAGE,
		DATA_PAGE,
		PROFILE_PAGE,

		LCD_SCREEN_STATE_SIZE,
	};

	static void SetLcdScreenState(const LcdScreenState state)
	{
		GetInstance()->m_lcd_screen_state = state;
	}

	static void SetLcdPause(const bool flag)
	{
		GetInstance()->m_is_lcd_pause = flag;
	}

	static LcdScreenState GetLcdScreenState()
	{
		return GetInstance()->m_lcd_screen_state;
	}

	static bool IsLcdPause()
	{
		return GetInstance()->m_is_lcd_pause;
	}

	static constexpr int GetAutoStopTime()
	{
		return 0;
	}

	static constexpr int GetEmergencyStopDelay()
	{
		return 3;
	}

	static constexpr int GetBrakeInterval()
	{
		return 19;
	}

	static constexpr int GetLedInterval()
	{
		return 250;
	}

	static constexpr int GetSpeedInterval()
	{
		return 19;
	}

	static constexpr int GetTurnThreshold()
	{
		return 38;
	}

	static constexpr int GetTurnInterval()
	{
		return 7;
	}

	static constexpr int GetJoystickInterval()
	{
		return 25;
	}

	static constexpr int GetCcdMid()
	{
		return 60;
	}

	static constexpr int GetCcdValidPixel()
	{
		return 128 - 20;
	}

	static constexpr int GetCcdValidPixelOffset()
	{
		return 10;
	}

	static int GetCcdThreshold(const uint8_t id)
	{
		switch (id)
		{
		default:
		case 0:
			// 21ms
			//return 0x520;
			//return 0x310;
			//return 0x260;
			//return 237;
			//return 260;
			return 229;

		//case 1:
			//return 0x240;
			//return 0x310;
		}
	}

private:
	Config();

	static Config* GetInstance()
	{
		if (!m_instance)
		{
			m_instance = new Config;
		}
		return m_instance;
	}

	LcdScreenState m_lcd_screen_state;
	bool m_is_lcd_pause;

	static Config *m_instance;
};

}

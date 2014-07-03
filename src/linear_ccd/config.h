/*
 * config.h
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

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

	static constexpr int GetTurnThreshold()
	{
		return 36;
	}

	static constexpr int GetCcdValidPixel()
	{
		return 128 - 10;
	}

	static constexpr int GetCcdValidPixelOffset()
	{
		return 5;
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

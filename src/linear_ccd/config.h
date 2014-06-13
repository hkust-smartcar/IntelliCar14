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
		NULL_PAGE,
		LCD_SCREEN_STATE_SIZE,
	};

	static void SetLcdScreenState(const LcdScreenState state)
	{
		GetInstance()->m_lcd_screen_state = state;
	}

	static LcdScreenState GetLcdScreenState()
	{
		return GetInstance()->m_lcd_screen_state;
	}

	static constexpr int GetTurnThreshold()
	{
		return TURN_THRESHOLD;
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
	static constexpr int TURN_THRESHOLD = 36;

	static Config *m_instance;
};

}

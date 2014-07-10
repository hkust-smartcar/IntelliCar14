/*
 * linear_ccd_app.cpp
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <libsc/k60/joystick.h>
#include <libsc/com/lcd.h>
#include <libsc/com/lcd_console.h>
#include <libsc/k60/simple_buzzer.h>
#include <libsc/k60/system.h>
#include <libsc/k60/timer.h>
#include <libsc/k60/trs_d05.h>
#include <libutil/misc.h>

#include "linear_ccd/config.h"
#include "linear_ccd/auto_program.h"
#include "linear_ccd/auto_program_2.h"
#include "linear_ccd/calibrate_program.h"
#include "linear_ccd/linear_ccd_app.h"

using namespace libsc;
using namespace libsc::k60;

namespace linear_ccd
{

namespace
{

class ProgramMenu
{
public:
	ProgramMenu();

	void Select(const int id);

	int GetSelectedId() const
	{
		return m_select;
	}

	Program::Token GetSelectedItem() const;

private:
	void Redraw();

	Lcd m_lcd;
	LcdConsole m_console;

	int m_select;
};

ProgramMenu::ProgramMenu()
		: m_lcd(true),
		  m_console(&m_lcd),
		  m_select(0)
{
	Redraw();
}

void ProgramMenu::Select(const int id)
{
	m_select = libutil::Clamp<int>(0, id, (int)Program::Token::SIZE - 1);
	Redraw();
}

Program::Token ProgramMenu::GetSelectedItem() const
{
	switch (m_select)
	{
	default:
	case 0:
		return Program::Token::AUTO2;

	case 1:
		return Program::Token::AUTO;

	case 2:
		return Program::Token::CALIBRATE;
	}
}

void ProgramMenu::Redraw()
{
	m_console.SetCursorRow(0);
	m_console.PrintString("Auto 2\n", 0xFFFF, (m_select == 0) ? 0x35BC : 0);
	m_console.PrintString("Auto 1\n", 0xFFFF, (m_select == 1) ? 0x35BC : 0);
	m_console.PrintString("Calibrate\n", 0xFFFF, (m_select == 2) ? 0x35BC : 0);
}

}

void LinearCcdApp::Run()
{
	System::Init();
	ServoProtect();
	Beep();

	switch (SelectProgram())
	{
	case Program::Token::AUTO2:
		{
			AutoProgram2 prog;
			prog.Run();
		}
		break;

	case Program::Token::AUTO:
		{
			AutoProgram prog;
			prog.Run();
		}
		break;

	case Program::Token::CALIBRATE:
		{
			CalibrateProgram prog;
			prog.Run();
		}
		break;

	case Program::Token::SIZE:
		break;
	}
}

Program::Token LinearCcdApp::SelectProgram()
{
	ProgramMenu menu;
	Joystick joystick(0);

	int delay = 0;
	Timer::TimerInt time = System::Time();
	while (true)
	{
		const Timer::TimerInt now = System::Time();
		if (Timer::TimeDiff(now, time) >= 10)
		{
			if (delay > 0)
			{
				--delay;
			}
			else
			{
				switch (joystick.GetState())
				{
				case Joystick::State::UP:
					menu.Select(menu.GetSelectedId() - 1);
					break;

				case Joystick::State::DOWN:
					menu.Select(menu.GetSelectedId() + 1);
					break;

				case Joystick::State::SELECT:
					return menu.GetSelectedItem();

				default:
					break;
				}
				delay = 20;
			}

			time = now;
		}
	}
}

void LinearCcdApp::Beep()
{
	SimpleBuzzer buzzer(0);
	buzzer.SetBeep(true);
	System::DelayMs(100);
	buzzer.SetBeep(false);
}

void LinearCcdApp::ServoProtect()
{
	TrsD05 s(0);
	System::DelayMs(250);
}

}

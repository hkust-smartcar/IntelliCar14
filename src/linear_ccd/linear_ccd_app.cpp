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
#include "linear_ccd/auto_program_3.h"
#include "linear_ccd/auto_program_4.h"
#include "linear_ccd/calibrate_program.h"
#include "linear_ccd/calibrate_program_2.h"
#include "linear_ccd/linear_ccd_app.h"
#include "linear_ccd/manual_program_3.h"
#include "linear_ccd/manual_program_4.h"

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
	if (m_select >= 0 && m_select < (int)Program::Token::SIZE)
	{
		return (Program::Token)m_select;
	}
	else
	{
		return (Program::Token)0;
	}
}

void ProgramMenu::Redraw()
{
	m_console.SetCursorRow(0);
	m_console.PrintString("Auto 4\n", 0xFFFF, (m_select == 0) ? 0x35BC : 0);
	m_console.PrintString("Auto 3\n", 0xFFFF, (m_select == 1) ? 0x35BC : 0);
	m_console.PrintString("Auto 2\n", 0xFFFF, (m_select == 2) ? 0x35BC : 0);
	m_console.PrintString("Auto 1\n", 0xFFFF, (m_select == 3) ? 0x35BC : 0);
	m_console.PrintString("Manual 4\n", 0xFFFF, (m_select == 4) ? 0x35BC : 0);
	m_console.PrintString("Manual 3\n", 0xFFFF, (m_select == 5) ? 0x35BC : 0);
	m_console.PrintString("Calibrate 2\n", 0xFFFF, (m_select == 6) ? 0x35BC : 0);
	m_console.PrintString("Calibrate\n", 0xFFFF, (m_select == 7) ? 0x35BC : 0);
}

}

void LinearCcdApp::Run()
{
	System::Init();
	ServoProtect();
	Beep();

	switch (SelectProgram())
	{
	case Program::Token::AUTO4:
		{
			AutoProgram4 prog;
			prog.Run();
		}
		break;

	case Program::Token::AUTO3:
		{
			AutoProgram3 prog;
			prog.Run();
		}
		break;

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

	case Program::Token::MANUAL4:
		{
			ManualProgram4 prog;
			prog.Run();
		}
		break;

	case Program::Token::MANUAL3:
		{
			ManualProgram3 prog;
			prog.Run();
		}
		break;

	case Program::Token::CALIBRATE:
		{
			CalibrateProgram prog;
			prog.Run();
		}
		break;

	case Program::Token::CALIBRATE2:
		{
			CalibrateProgram2 prog;
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
		if (Timer::TimeDiff(now, time) >= 5)
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

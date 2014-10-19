/*
 * auto_program.cpp
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdint>
#include <cstdlib>

#include <bitset>
#include <string>

#include <libbase/log.h>
#include <libbase/syscall.h>
#include <libbase/k60/vectors.h>

#include <libsc/k60/linear_ccd.h>
#include <libsc/k60/system.h>
#include <libsc/k60/timer.h>
#include <libutil/misc.h>
#include <libutil/string.h>

#include "linear_ccd/debug.h"
#include "linear_ccd/config.h"
#include "linear_ccd/auto_program.h"
#include "linear_ccd/beep_manager.h"
#include "linear_ccd/car.h"
#include "linear_ccd/dir_control_algorithm.h"
#include "linear_ccd/median_ccd_filter.h"
#include "linear_ccd/speed_control_1.h"
#include "linear_ccd/turn_hint.h"

using namespace libbase::k60;
using namespace libsc::k60;
using namespace std;

#define LED_FREQ Config::GetLedInterval()
#define SERVO_FREQ Config::GetTurnInterval()
#define SPEED_CTRL_FREQ Config::GetSpeedInterval()
#define JOYSTICK_FREQ Config::GetJoystickInterval()

#define EMERGENCY_STOP_DELAY Config::GetEmergencyStopDelay()

#define AUTOSTOP_TIME Config::GetAutoStopTime()

namespace linear_ccd
{

AutoProgram *AutoProgram::m_instance = nullptr;

AutoProgram::AutoProgram()
		: m_ccd_filter(2),
		  m_dir_control({DirControlAlgorithm(&m_car), DirControlAlgorithm(&m_car)}),
		  m_speed_control(&m_car),
		  m_start(0),
		  m_is_stop(false),
		  m_is_turn(false),
		  m_mode(0)
{
	m_instance = this;
}

AutoProgram::~AutoProgram()
{
	m_instance = nullptr;
}

void AutoProgram::Run()
{
	g_fwrite_handler = FwriteHandler;
	g_hard_fault_handler = HardFaultHandler;
	InitialStage();

	while (true)
	{
		ServoPass();
		//DetectStopLine();
		if (!m_is_stop)
		{
			if (Config::GetAutoStopTime() != 0
					&& Timer::TimeDiff(System::Time(), m_start)
							>= Config::GetAutoStopTime())
			{
				m_is_stop = true;
			}
			else
			{
				SpeedControlPass();
			}
			DetectEmergencyStop();
		}
		else
		{
			m_car.StopMotor();
		}
		LedPass();
		JoystickPass();
		m_car.GetBeepManager()->Process();
	}
}

void AutoProgram::InitialStage()
{
	iprintf("Initialize\n");
	m_car.GetBeepManager()->Beep(100);

#ifdef K60_2014_CCD
	//dac_init(DAC0);
	//dac_out(DAC0, Config::GetCcdThreshold(0));
	//dac_init(DAC1);
	//dac_out(DAC1, Config::GetCcdThreshold(1));
	m_car.SetCcdDacThreshold(Config::GetCcdThreshold(0));

	const Timer::TimerInt time = System::Time();
	while (Timer::TimeDiff(System::Time(), time) < INITIAL_DELAY)
	{
		const bitset<2> &buttons = m_car.GetButtonState();
		if (buttons[0])
		{
			const bitset<5> &switches = m_car.GetSwitchState();
			for (int i = 0; i < 5; ++i)
			{
				if (switches[i])
				{
					m_mode = i + 1;
					if (i == 4)
					{
						m_car.SwitchLed(0, true);
						m_car.SwitchLed(3, true);
					}
					else
					{
						m_car.SwitchLed(i, true);
					}
				}
				else
				{
					if (i < 4)
					{
						m_car.SwitchLed(i, false);
					}
				}
			}
		}

		m_car.UpdateGyro();
		JoystickPass();
		m_car.GetBeepManager()->Process();
	}

#else
	while (Clock::Time() < INITIAL_DELAY)
	{
		const bitset<4> &buttons = m_car.GetButtonState();
		for (int i = 0; i < 4; ++i)
		{
			if (buttons[i])
			{
				m_mode = i + 1;
				for (int j = 0; j < 4; ++j)
				{
					m_car.SwitchLed(j, (i == j));
				}
				break;
			}
		}
		m_car.UpdateGyro();
		BeepManager::GetInstance(&m_car)->Process();
	}

#endif

	if (m_mode == 0)
	{
		const bitset<5> &switches = m_car.GetSwitchState();
		for (int i = 0; i < 5; ++i)
		{
			if (switches[i])
			{
				m_dir_control[0].SetMode(i + 1);
				m_dir_control[1].SetMode(i + 1 + 5);
			}
		}
	}
	else
	{
		m_dir_control[0].SetMode(m_mode);
		m_dir_control[1].SetMode(m_mode + 5);
	}
	m_speed_control.SetMode(m_mode);

	// Reset encoder count
	m_car.UpdateEncoder();

	m_dir_control[0].OnFinishWarmUp(&m_car);
	m_dir_control[1].OnFinishWarmUp(&m_car);
	m_speed_control.OnFinishWarmUp();

	// Dump first CCD sample
	for (int i = 0; i < LinearCcd::kSensorW; ++i)
	{
		m_car.CcdSampleProcess();
	}
	m_car.StartCcdSample();
	m_servo_state.prev_run = System::Time();

	m_start = System::Time();
	m_car.GetBeepManager()->Beep(100);
	iprintf("Start\n");
}

void AutoProgram::LedPass()
{
	const Timer::TimerInt time = System::Time();
	if (Timer::TimeDiff(time, m_led_state.prev_run) >= LED_FREQ)
	{
		if (m_mode == 5)
		{
			m_car.SwitchLed(0, m_led_state.flag);
			m_car.SwitchLed(3, m_led_state.flag);
		}
		else if (m_mode > 0)
		{
			m_car.SwitchLed(m_mode - 1, m_led_state.flag);
		}
		else
		{
			for (int i = 0; i < 4; ++i)
			{
				m_car.SwitchLed(i, m_led_state.flag);
			}
		}
		m_led_state.flag ^= true;

		m_led_state.prev_run = time - (time % LED_FREQ);
	}
}

void AutoProgram::ServoPass()
{
	const Timer::TimerInt time = System::Time();
	if (Timer::TimeDiff(time, m_servo_state.prev_run) >= SERVO_FREQ
			&& m_car.IsCcdReady())
	{
#ifdef DEBUG_PRINT_INTERVAL
		iprintf("ccd f: %lu\n", Timer::TimeDiff(time, m_servo_state.prev_run));
#endif
#ifdef DEBUG_LCD_PRINT_INTERVAL
		if (Config::GetLcdScreenState() == Config::PROFILE_PAGE
				&& !Config::IsLcdPause())
		{
			m_car.LcdSetRow(0);
			m_car.LcdPrintString(libutil::String::Format("ccd f: %d\n",
					Timer::TimeDiff(time, m_servo_state.prev_run)).c_str(),
					0xFFFF);
		}
#endif

		m_servo_state.prev_run = time;
		m_car.StartCcdSample();

/*
		//const int16_t up_turn = 0;
		const bitset<LinearCcd::kSensorW> &ccd_data_up = m_ccd_filter.Filter(
				m_car.GetCcdSample(1));
		const int16_t up_turn = m_dir_control[1].Process(ccd_data_up);
#ifdef DEBUG_PRINT_CASE
		LOG_D("up_case :%d", m_dir_control[1].GetCase());
#endif
#ifdef DEBUG_LCD_PRINT_CASE
		if (Config::GetLcdScreenState() == Config::DATA_PAGE)
		{
			m_car.LcdSetRow(1);
			m_car.LcdPrintString(libutil::String::Format("UP c: %d\nm: %d\n",
					m_dir_control[1].GetCase(),
					m_dir_control[1].GetMid()).c_str(), 0xFFFF);
		}
#endif
#ifdef DEBUG_LCD_PRINT_EDGE
		if (Config::GetLcdScreenState() == Config::DATA_PAGE)
		{
			m_car.LcdSetRow(4);
			m_car.LcdPrintString(libutil::String::Format("e: %3d %3d\n",
					m_dir_control[1].GetCurrLeftEdge(),
					m_dir_control[1].GetCurrRightEdge()).c_str(), 0xFFFF);
		}
#endif

		if (abs(up_turn) > Config::GetTurnThreshold())
		{
			m_dir_control[0].SetTurnHint(TurnHint::PRE_TURN);
		}
*/
		const bitset<LinearCcd::kSensorW> ccd_data_up;
		const int16_t up_turn = 0;

		const bitset<LinearCcd::kSensorW> &ccd_data_down = m_ccd_filter.Filter(
				m_car.GetCcdSample(0));
		const int16_t down_turn = m_dir_control[0].Process(ccd_data_down);
#ifdef DEBUG_PRINT_CASE
		LOG_D("down_case :%d", m_dir_control[0].GetCase());
#endif
#ifdef DEBUG_LCD_PRINT_CASE
		if (Config::GetLcdScreenState() == Config::DATA_PAGE
				&& !Config::IsLcdPause())
		{
			m_car.LcdSetRow(5);
			m_car.LcdPrintString(libutil::String::Format("DN c: %d\nm: %d\n",
					m_dir_control[0].GetCase(),
					m_dir_control[0].GetMid()).c_str(), 0xFFFF);
		}
#endif
#ifdef DEBUG_LCD_PRINT_EDGE
		if (Config::GetLcdScreenState() == Config::DATA_PAGE
				&& !Config::IsLcdPause())
		{
			m_car.LcdSetRow(8);
			m_car.LcdPrintString(libutil::String::Format("e: %3d %3d\n",
					m_dir_control[0].GetCurrLeftEdge(),
					m_dir_control[0].GetCurrRightEdge()).c_str(), 0xFFFF);
		}
#endif
#ifdef DEBUG_PRINT_EDGE
		m_car.UartSendStr(libutil::String::Format("e: %3d %3d\n",
				m_dir_control[0].GetCurrLeftEdge(),
				m_dir_control[0].GetCurrRightEdge()));
#endif
#ifdef DEBUG_BEEP_CCD_FILL
		if (m_dir_control[0].IsAllBlack() || m_dir_control[0].IsAllWhite())
		{
			BeepManager::GetInstance(&m_car)->Beep(100);
		}
#endif

		const int16_t turning = ConcludeTurning(up_turn, down_turn);
#ifdef DEBUG_PRINT_TURNING
		LOG_D("turn :%d", turning);
#endif
#ifdef DEBUG_LCD_PRINT_TURNING
		if (Config::GetLcdScreenState() == Config::DATA_PAGE
				&& !Config::IsLcdPause())
		{
			m_car.LcdSetRow(0);
			m_car.LcdPrintString(libutil::String::Format("t: %d\n", turning)
					.c_str(), 0xFFFF);
			m_car.LcdSetRow(3);
			m_car.LcdPrintString(libutil::String::Format("t: %d\n", up_turn)
					.c_str(), 0xFFFF);
			m_car.LcdSetRow(7);
			m_car.LcdPrintString(libutil::String::Format("t: %d\n", down_turn)
					.c_str(), 0xFFFF);
		}
#endif

#ifdef DEBUG_PRINT_ERROR_AND_PID
		static int print_error_pid_delay = 0;
		if (++print_error_pid_delay >= 5)
		{
			printf("%d %ld %.3f %.3f %d\n", 64 - m_dir_control[0].GetMid(),
					m_dir_control[0].GetSetpoint() - m_dir_control[0].GetMid(),
					m_dir_control[0].GetP(), m_dir_control[0].GetD(), -turning);
			print_error_pid_delay = 0;
		}
#endif

		m_car.SetTurning(turning);

		if (abs(down_turn) <= Config::GetTurnThreshold())
		{
/*
			if (abs(up_turn) > Config::GetTurnThreshold()
					|| (m_is_turn && (m_dir_control[1].IsAllBlack()
							|| m_dir_control[1].IsAllWhite())))
			{
				m_speed_control.SetTurnHint(TurnHint::PRE_TURN);
			}
			else
			{
				m_is_turn = false;
				m_speed_control.SetTurnHint(TurnHint::STRAIGHT);
			}
*/
			m_is_turn = false;
			m_speed_control.SetTurnHint(TurnHint::STRAIGHT);
		}
		else
		{
			// Prevent seeing crazy stuff while turning
			m_dir_control[1].ResetMid();
			m_is_turn = true;
			m_speed_control.SetTurnHint(TurnHint::TURN);
		}

#ifdef DEBUG_LCD_PRINT_CCD
		if (Config::GetLcdScreenState() == Config::CCD_PAGE
				&& !Config::IsLcdPause())
		{
			static int y = 0;
			static const int MID_Y = St7735r::kH / 2;
			const uint8_t buf_size = LinearCcd::kSensorW;
			uint8_t buf[buf_size] = {};
			/*
			for (int i = 0; i < LinearCcd::kSensorW; ++i)
			{
				buf[i] = ccd_data_up[i] ? 0xFF : 0x00;
			}
			m_car.LcdDrawGrayscalePixelBuffer(0, y, buf_size, 1, buf);
			if (m_dir_control[1].GetMid() >= 0
					&& m_dir_control[1].GetMid() < LinearCcd::kSensorW)
			{
				m_car.LcdDrawPixel(m_dir_control[1].GetMid(), y + MID_Y,
						libutil::GetRgb565(0xFF, 0, 0x33));
			}
			*/
			for (int i = 0; i < LinearCcd::kSensorW; ++i)
			{
				buf[i] = ccd_data_down[i] ? 0xFF : 0x00;
			}
			m_car.LcdDrawGrayscalePixelBuffer(0, y + MID_Y, buf_size, 1, buf);
			if (m_dir_control[0].GetMid() >= 0
					&& m_dir_control[0].GetMid() < LinearCcd::kSensorW)
			{
				m_car.LcdDrawPixel(m_dir_control[0].GetMid(), y + MID_Y,
						libutil::GetRgb565(0xE5, 0x33, 0xB5));
			}

			if (y++ >= MID_Y)
			{
				y = 0;
			}
		}
#endif

#ifdef DEBUG_PRINT_CCD
		// Send CCD data through UART
		string str;
		str.reserve(LinearCcd::kSensorW + 1);
		for (int i = 3; i < LinearCcd::kSensorW - 3; ++i)
		{
			str += ccd_data[i] ? '#' : '.';
		}
		str += '\n';
		m_car.UartSendStr(std::move(str));
#endif

#ifdef DEBUG_PRINT_INTERVAL
		iprintf("ccd t: %lu\n", Timer::TimeDiff(System::Time(), time));
#endif
#ifdef DEBUG_LCD_PRINT_INTERVAL
		if (Config::GetLcdScreenState() == Config::PROFILE_PAGE
				&& !Config::IsLcdPause())
		{
			m_car.LcdSetRow(1);
			m_car.LcdPrintString(libutil::String::Format("ccd t: %d\n",
					Timer::TimeDiff(System::Time(), time)).c_str(), 0xFFFF);
		}
#endif
	}

	m_car.CcdSampleProcess();
}

void AutoProgram::SpeedControlPass()
{
	const Timer::TimerInt time = System::Time();
	if (Timer::TimeDiff(time, m_speed_state.prev_run) >= SPEED_CTRL_FREQ)
	{
#ifdef DEBUG_PRINT_INTERVAL
		iprintf("speed freq: %lu\n", Timer::TimeDiff(time, m_speed_state.prev_run));
#endif
#ifdef DEBUG_LCD_PRINT_INTERVAL
		if (Config::GetLcdScreenState() == Config::PROFILE_PAGE
				&& !Config::IsLcdPause())
		{
			m_car.LcdSetRow(2);
			m_car.LcdPrintString(libutil::String::Format("spd freg: %d\n",
					Timer::TimeDiff(time, m_speed_state.prev_run)).c_str(),
					0xFFFF);
		}
#endif

		m_speed_state.prev_run = time;

		m_speed_control.Control();

#ifdef DEBUG_PRINT_INTERVAL
		iprintf("spd t: %lu\n", Timer::TimeDiff(System::Time(), time));
#endif
#ifdef DEBUG_LCD_PRINT_INTERVAL
		if (Config::GetLcdScreenState() == Config::PROFILE_PAGE
				&& !Config::IsLcdPause())
		{
			m_car.LcdSetRow(3);
			m_car.LcdPrintString(libutil::String::Format("spd t: %d\n",
					Timer::TimeDiff(System::Time(), time)).c_str(), 0xFFFF);
		}
#endif
	}
}

void AutoProgram::JoystickPass()
{
	const Timer::TimerInt time = System::Time();
	if (Timer::TimeDiff(time, m_joystick_state.prev_run) >= JOYSTICK_FREQ)
	{
		if (m_joystick_state.delay > 0)
		{
			--m_joystick_state.delay;
		}
		else
		{
			switch (m_car.GetJoystickState())
			{
			case Joystick::State::LEFT:
				if (Config::GetLcdScreenState() - 1 < 0)
				{
					Config::SetLcdScreenState((Config::LcdScreenState)
							(Config::LCD_SCREEN_STATE_SIZE - 1));
				}
				else
				{
					Config::SetLcdScreenState((Config::LcdScreenState)
							(Config::GetLcdScreenState() - 1));
				}
				m_car.LcdClear();
				m_car.LcdSetRow(0);
				m_car.LcdPrintString(libutil::String::Format("%d",
						Config::GetLcdScreenState()).c_str(), 0xFFFF);
				break;

			case Joystick::State::RIGHT:
				if (Config::GetLcdScreenState() + 1
						>= Config::LCD_SCREEN_STATE_SIZE)
				{
					Config::SetLcdScreenState((Config::LcdScreenState)0);
				}
				else
				{
					Config::SetLcdScreenState((Config::LcdScreenState)
							(Config::GetLcdScreenState() + 1));
				}
				m_car.LcdClear();
				m_car.LcdSetRow(0);
				m_car.LcdPrintString(libutil::String::Format("%d",
						Config::GetLcdScreenState()).c_str(), 0xFFFF);
				break;

			case Joystick::State::SELECT:
				Config::SetLcdPause(!Config::IsLcdPause());
				break;

			default:
				break;
			}
			m_joystick_state.delay = 10;
		}

		m_joystick_state.prev_run = time - (time % JOYSTICK_FREQ);
	}
}

int AutoProgram::FwriteHandler(int, char *ptr, int len)
{
	if (m_instance)
	{
		m_instance->m_car.UartSendBuffer((const Byte*)ptr, len);
	}
	return len;
}

void AutoProgram::HardFaultHandler()
{
	if (m_instance)
	{
		m_instance->m_car.StopMotor();
		for (int i = 0; 0 < 4; ++i)
		{
			m_instance->m_car.SwitchLed(i, true);
		}
	}
	while(true)
	{}
}

void AutoProgram::DetectStopLine()
{
	if (!m_car.IsLightSensorDetected(0) && !m_car.IsLightSensorDetected(1))
	{
		//m_is_stop = true;
	}
}

void AutoProgram::DetectEmergencyStop()
{
	static bool is_startup = true;
	const Timer::TimerInt time = System::Time();
	if (is_startup && Timer::TimeDiff(time, m_start) > 2000)
	{
		is_startup = false;
	}

	const int16_t count = m_car.GetEncoderCount();
	if (!is_startup && abs(count) < 30)
	{
		if (m_emergency_stop_state.is_triggered)
		{
			if (Timer::TimeDiff(time, m_emergency_stop_state.trigger_time) > 200)
			{
				// Emergency stop
				m_is_stop = true;
				if (m_mode != 0)
				{
					m_car.SetBuzzerBeep(true);
				}
			}
		}
		else
		{
			m_emergency_stop_state.is_triggered = true;
			m_emergency_stop_state.trigger_time = time;
		}
	}
	else
	{
		m_emergency_stop_state.is_triggered = false;
	}
}

int16_t AutoProgram::ConcludeTurning(const int16_t,
		const int16_t down_turn) const
{
	return down_turn;
	/*
	if (abs(up_turn) > Config::GetTurnThreshold()
			&& abs(down_turn) < Config::GetTurnThreshold())
	{
		// Going to turn
		return (up_turn + down_turn) / 2;
	}
	else
	{
		return down_turn;
	}
	*/
	//return up_turn + down_turn;
}

}

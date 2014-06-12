/*
 * linear_ccd_app.cpp
 * Linear CCD App
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <mini_common.h>
#include <hw_common.h>
#include <syscall.h>

#include <cstdint>
#include <cstdlib>
#include <bitset>

#include <log.h>
#include <MK60_dac.h>
#include <MK60_gpio.h>

#include "linear_ccd/debug.h"

#include <libsc/com/joystick.h>
#include <libsc/com/linear_ccd.h>
#include <libutil/clock.h>
#include <libutil/misc.h>
#include <libutil/string.h>

#include "linear_ccd/config.h"
#include "linear_ccd/bt_controller.h"
#include "linear_ccd/car.h"
#include "linear_ccd/linear_ccd_app.h"

using namespace std;
using libutil::Clock;

#define LED_FREQ 250
#define SERVO_FREQ 15
#define SPEED_CTRL_FREQ 19
#define JOYSTICK_FREQ 25

namespace linear_ccd
{

LinearCcdApp *LinearCcdApp::m_instance = nullptr;

LinearCcdApp::LinearCcdApp()
		: m_dir_control({DirControlAlgorithm(&m_car), DirControlAlgorithm(&m_car)}),
		  m_is_stop(false), m_mode(0), m_is_manual_interruptted(false)
{
	m_instance = this;
}

LinearCcdApp::~LinearCcdApp()
{
	m_instance = nullptr;
}

void LinearCcdApp::Run()
{
	__g_fwrite_handler = FwriteHandler;
	Clock::Init();
	InitialStage();

	int servo = 0;
	while (true)
	{
#ifdef DEBUG_MANUAL_CONTROL
		if (BtControlPass())
		{
			m_is_manual_interruptted = true;
		}
		if (m_is_manual_interruptted)
		{
			continue;
		}
#endif
		/*
		if (++servo > 100)
		{
			servo = 0;
		}
		m_car.SetRightPercentage(servo);
		DELAY_MS(25);
		*/
		ServoPass();
		//DetectStopLine();
		DetectEmergencyStop();
		if (!m_is_stop)
		{
			SpeedControlPass();
		}
		else
		{
			m_car.StopMotor();
		}
		LedPass();
		JoystickPass();
	}
}

void LinearCcdApp::InitialStage()
{
#ifdef LINEAR_CCD_2014
	dac_init(DAC0);
	//dac_out(DAC0, 0x520); 21freq
	dac_out(DAC0, 0x440);
	dac_init(DAC1);
	dac_out(DAC1, 0x420);

	while (Clock::Time() < INITIAL_DELAY)
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
	}

#endif

	m_dir_control[0].SetMode(m_mode);
	m_dir_control[1].SetMode(m_mode + 5);
	m_speed_control.SetMode(m_mode);

	// Reset encoder count
	m_car.UpdateEncoder();

	m_dir_control[0].OnFinishWarmUp(&m_car);
	m_dir_control[1].OnFinishWarmUp(&m_car);
	m_speed_control.OnFinishWarmUp(&m_car);

	// Dump first CCD sample
	for (int i = 0; i < libsc::LinearCcd::SENSOR_W; ++i)
	{
		m_car.CcdSampleProcess(0);
		//m_car.CcdSampleProcess(1);
	}
	m_car.StartCcdSample(0);
	//m_car.StartCcdSample(1);
	m_servo_state.prev_run = Clock::Time();
}

void LinearCcdApp::LedPass()
{
	const Clock::ClockInt time = Clock::Time();
	if (Clock::TimeDiff(time, m_led_state.prev_run) >= LED_FREQ)
	{
		if (m_mode > 0)
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

void LinearCcdApp::ServoPass()
{
	const Clock::ClockInt time = Clock::Time();
	if (Clock::TimeDiff(time, m_servo_state.prev_run) >= SERVO_FREQ
			&& m_car.IsCcdReady(0) /*&& m_car.IsCcdReady(1)*/)
	{
#ifdef DEBUG_PRINT_INTERVAL
		iprintf("ccd f: %d\n", Clock::TimeDiff(time, m_servo_state.prev_run));
#endif
#ifdef DEBUG_LCD_PRINT_INTERVAL
		if (Config::GetLcdScreenState() == Config::PROFILE_PAGE)
		{
			m_car.LcdSetRow(0);
			m_car.LcdPrintString(libutil::String::Format("ccd f: %d\n",
					Clock::TimeDiff(time, m_servo_state.prev_run)).c_str(),
					0xFFFF);
		}
#endif

		m_servo_state.prev_run = time;
		m_car.StartCcdSample(0);
		//m_car.StartCcdSample(1);

		const int16_t up_turn = 0;
/*
		const bitset<libsc::LinearCcd::SENSOR_W> &ccd_data_up = FilterCcdData(
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
*/

		const bitset<libsc::LinearCcd::SENSOR_W> &ccd_data_down = FilterCcdData(
				m_car.GetCcdSample(0));
		const int16_t down_turn = m_dir_control[0].Process(ccd_data_down);
#ifdef DEBUG_PRINT_CASE
	LOG_D("down_case :%d", m_dir_control[0].GetCase());
#endif
#ifdef DEBUG_LCD_PRINT_CASE
	if (Config::GetLcdScreenState() == Config::DATA_PAGE)
	{
		m_car.LcdSetRow(4);
		m_car.LcdPrintString(libutil::String::Format("DN c: %d\nm: %d\n",
				m_dir_control[0].GetCase(),
				m_dir_control[0].GetMid()).c_str(), 0xFFFF);
	}
#endif

		const int16_t turning = ConcludeTurning(up_turn, down_turn);
#ifdef DEBUG_PRINT_TURNING
	LOG_D("turn :%d", turning);
#endif
#ifdef DEBUG_LCD_PRINT_TURNING
		if (Config::GetLcdScreenState() == Config::DATA_PAGE)
		{
			m_car.LcdSetRow(0);
			m_car.LcdPrintString(libutil::String::Format("t: %d\n", turning)
					.c_str(), 0xFFFF);
/*
			m_car.LcdSetRow(3);
			m_car.LcdPrintString(libutil::String::Format("t: %d\n", up_turn)
					.c_str(), 0xFFFF);
*/
			m_car.LcdSetRow(6);
			m_car.LcdPrintString(libutil::String::Format("t: %d\n", down_turn)
					.c_str(), 0xFFFF);
		}
#endif
		m_car.SetTurning(turning);

#ifdef DEBUG_LCD_PRINT_CCD
		if (Config::GetLcdScreenState() == Config::CCD_PAGE)
		{
			static int y = 0;
			static const int MID_Y = libsc::Lcd::H / 2;
			const uint8_t buf_size = libsc::LinearCcd::SENSOR_W;
			uint8_t buf[buf_size] = {};
/*
			for (int i = 0; i < libsc::LinearCcd::SENSOR_W; ++i)
			{
				buf[i] = ccd_data_up[i] ? 0xFF : 0x00;
			}
			m_car.LcdDrawGrayscalePixelBuffer(0, y, buf_size, 1, buf);
*/
			for (int i = 0; i < libsc::LinearCcd::SENSOR_W; ++i)
			{
				buf[i] = ccd_data_down[i] ? 0xFF : 0x00;
			}
			m_car.LcdDrawGrayscalePixelBuffer(0, y + MID_Y, buf_size, 1, buf);
			if (y++ >= MID_Y)
			{
				y = 0;
			}
		}
#endif

#ifdef DEBUG_PRINT_CCD
		// Send CCD data through UART
		char str[libsc::LinearCcd::SENSOR_W];
		for (int i = 3; i < libsc::LinearCcd::SENSOR_W - 3; ++i)
		{
			str[i] = ccd_data[i] ? '#' : '.';
		}
		m_car.UartSendBuffer((uint8_t*)(str + 3), libsc::LinearCcd::SENSOR_W - 6);
		m_car.UartSendStr("\n");
#endif

#ifdef DEBUG_PRINT_INTERVAL
		iprintf("ccd t: %d\n", Clock::TimeDiff(Clock::Time(), time));
#endif
#ifdef DEBUG_LCD_PRINT_INTERVAL
		if (Config::GetLcdScreenState() == Config::PROFILE_PAGE)
		{
			m_car.LcdSetRow(1);
			m_car.LcdPrintString(libutil::String::Format("ccd t: %d\n",
					Clock::TimeDiff(Clock::Time(), time)).c_str(), 0xFFFF);
		}
#endif
	}

	m_car.CcdSampleProcess(0);
	//m_car.CcdSampleProcess(1);
}

void LinearCcdApp::SpeedControlPass()
{
	const Clock::ClockInt time = Clock::Time();
	if (Clock::TimeDiff(time, m_speed_state.prev_run) >= SPEED_CTRL_FREQ)
	{
#ifdef DEBUG_PRINT_INTERVAL
		iprintf("speed freq: %d\n", Clock::TimeDiff(time, m_speed_state.prev_run));
#endif
#ifdef DEBUG_LCD_PRINT_INTERVAL
		if (Config::GetLcdScreenState() == Config::PROFILE_PAGE)
		{
			m_car.LcdSetRow(2);
			m_car.LcdPrintString(libutil::String::Format("spd freg: %d\n",
					Clock::TimeDiff(time, m_speed_state.prev_run)).c_str(),
					0xFFFF);
		}
#endif

		m_speed_state.prev_run = time;

		m_speed_control.Control(&m_car);

#ifdef DEBUG_PRINT_INTERVAL
		iprintf("spd t: %d\n", Clock::TimeDiff(Clock::Time(), time));
#endif
#ifdef DEBUG_LCD_PRINT_INTERVAL
		if (Config::GetLcdScreenState() == Config::PROFILE_PAGE)
		{
			m_car.LcdSetRow(3);
			m_car.LcdPrintString(libutil::String::Format("spd t: %d\n",
					Clock::TimeDiff(Clock::Time(), time)).c_str(), 0xFFFF);
		}
#endif
	}
}

void LinearCcdApp::JoystickPass()
{
	const Clock::ClockInt time = Clock::Time();
	if (Clock::TimeDiff(time, m_joystick_state.prev_run) >= JOYSTICK_FREQ)
	{
		if (m_joystick_state.delay > 0)
		{
			--m_joystick_state.delay;
		}
		else
		{
			switch (m_car.GetJoystickState())
			{
			case libsc::Joystick::LEFT:
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
				m_car.LcdClear(0);
				m_car.LcdSetRow(0);
				m_car.LcdPrintString(libutil::String::Format("%d",
						Config::GetLcdScreenState()).c_str(), 0xFFFF);
				break;

			case libsc::Joystick::RIGHT:
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
				m_car.LcdClear(0);
				m_car.LcdSetRow(0);
				m_car.LcdPrintString(libutil::String::Format("%d",
						Config::GetLcdScreenState()).c_str(), 0xFFFF);
				break;

			default:
				break;
			}
			m_joystick_state.delay = 10;
		}

		m_joystick_state.prev_run = time - (time % JOYSTICK_FREQ);
	}
}

bool LinearCcdApp::BtControlPass()
{
	return m_bt_control.Control(&m_car);
}

int LinearCcdApp::FwriteHandler(int, char *ptr, int len)
{
	if (m_instance)
	{
		m_instance->m_car.UartSendBuffer((const uint8_t*)ptr, len);
	}
	return len;
}

void LinearCcdApp::DetectStopLine()
{
	if (!m_car.IsLightSensorDetected(0) && !m_car.IsLightSensorDetected(1))
	{
		m_is_stop = true;
	}
}

void LinearCcdApp::DetectEmergencyStop()
{
	static bool is_startup = true;
	const Clock::ClockInt time = Clock::Time();
	if (is_startup && time > 2000 + INITIAL_DELAY)
	{
		is_startup = false;
	}

	const int16_t count = m_car.GetEncoderCount();
	if (!is_startup && abs(count) < 30)
	{
		// Emergency stop
		m_is_stop = true;
	}
}

bitset<libsc::LinearCcd::SENSOR_W> LinearCcdApp::FilterCcdData(
		const bitset<libsc::LinearCcd::SENSOR_W> &data) const
{
	bitset<libsc::LinearCcd::SENSOR_W> result = data;
	constexpr int block_size = 1;
	// Simple alg'm to filter out peculiar color in the middle
	for (int i = block_size; i < libsc::LinearCcd::SENSOR_W - block_size; ++i)
	{
		int value = 0;
		for (int j = 0; j < block_size; ++j)
		{
			value += data[i - j - 1];
		}
		for (int j = 0; j < block_size; ++j)
		{
			value += data[i + j + 1];
		}
		const int threshold = ceilf((block_size * 2) * 0.65f);
		if (value >= threshold)
		{
			result[i] = true;
		}
		else if (value <= block_size * 2 - threshold)
		{
			result[i] = false;
		}
	}
	return result;
}

int16_t LinearCcdApp::ConcludeTurning(const int16_t up_turn,
		const int16_t down_turn) const
{
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
	return down_turn;
	//return up_turn + down_turn;
}

}

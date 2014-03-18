/*
 * car.h
 * Magnetic car
 *
 * Author:
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef MAGNETIC_CAR_H_
#define MAGNETIC_CAR_H_

#include <libsc/com/led.h>
#include <libsc/com/uart_device.h>

namespace magnetic
{

class Car
{
public:
	Car();

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

	void UartSendStr(const char *str)
	{
		m_uart.SendStr(str);
	}

	void UartSendBuffer(const uint8_t *buf, const uint32_t len)
	{
		m_uart.SendBuffer(buf, len);
	}

	bool UartPeekChar(char *out_ch)
	{
		return m_uart.PeekChar(out_ch);
	}

private:
	libsc::Led m_leds[4];
	libsc::UartDevice m_uart;
};

}

#endif /* MAGNETIC_CAR_H_ */

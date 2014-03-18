/*
 * car.cpp
 * Camera car
 *
 * Author:
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdint>

#include <libsc/com/led.h>
#include <libsc/com/uart_device.h>

#include "camera/car.h"

using namespace libsc;

namespace camera
{

Car::Car()
		: m_leds{Led(0), Led(1), Led(2), Led(3)}, m_uart(3, 115200)
{
	m_uart.StartReceive();
}

}

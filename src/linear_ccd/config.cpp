/*
 * config.cpp
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include "linear_ccd/config.h"

namespace linear_ccd
{

Config *Config::m_instance = nullptr;

Config::Config()
		: m_lcd_screen_state(CCD_PAGE)
{}

}

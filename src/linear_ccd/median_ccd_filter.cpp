/*
 * median_ccd_filter.cpp
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <bitset>

#include <libsc/k60/linear_ccd.h>

#include "linear_ccd/config.h"
#include "linear_ccd/median_ccd_filter.h"

using namespace libsc::k60;
using namespace std;

namespace linear_ccd
{

MedianCcdFilter::MedianCcdFilter(const int window)
		: m_window(window)
{
	// e.g, when window == 2, elements == 5, the 3rd one will be the middle
	m_mid = m_window + 1;
}

bitset<LinearCcd::SENSOR_W> MedianCcdFilter::Filter(
		const bitset<LinearCcd::SENSOR_W> &data)
{
	bitset<LinearCcd::SENSOR_W> result = data;
	// Apply median filter to reduce noise
	int x = 0;
	for (int i = 0; i < Config::GetCcdValidPixelOffset() - m_window; ++i)
	{
		const int index = i + Config::GetCcdValidPixelOffset();
		x += data[index];
		if (i >= m_window)
		{
			x -= data[index - m_window];
			result.set(index, (x >= m_mid));
		}
	}
	return result;
}

}

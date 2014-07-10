/*
 * median_ccd_filter.h
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_MEDIAN_CCD_FILTER_H_
#define LINEAR_CCD_MEDIAN_CCD_FILTER_H_

#include <bitset>

#include <libsc/k60/linear_ccd.h>

namespace linear_ccd
{

class MedianCcdFilter
{
public:
	explicit MedianCcdFilter(const int window);

	std::bitset<libsc::k60::LinearCcd::SENSOR_W> Filter(
			const std::bitset<libsc::k60::LinearCcd::SENSOR_W> &data);

private:
	int m_window;
	int m_mid;
};

}

#endif /* LINEAR_CCD_MEDIAN_CCD_FILTER_H_ */

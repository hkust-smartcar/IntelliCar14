/*
 * track_analyzer.cpp
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <bitset>

#include <libsc/k60/linear_ccd.h>
#include <libutil/misc.h>

#include "linear_ccd/config.h"
#include "linear_ccd/track_analyzer.h"

#define VALID_PIXEL Config::GetCcdValidPixel()
#define VALID_OFFSET Config::GetCcdValidPixelOffset()
#define CCD_MID_POS 64

using namespace libsc::k60;
using namespace std;

namespace linear_ccd
{

TrackAnalyzer::TrackAnalyzer(const int edge)
		: m_edge(edge),
		  m_prev_mid(CCD_MID_POS),
		  m_mid(CCD_MID_POS),
		  m_left_edge(-1),
		  m_right_edge(-1),
		  m_is_all_black(false),
		  m_is_all_white(false)
{}

void TrackAnalyzer::Analyze(const bitset<LinearCcd::SENSOR_W> &ccd_data)
{
	m_prev_mid = m_mid;
	m_mid = 0;
	m_left_edge = -1;
	m_right_edge = -1;

	if (IsFill(ccd_data))
	{
		m_mid = m_prev_mid;
		return;
	}

	DetectEdge(ccd_data);
	if (m_left_edge != -1 && m_right_edge != -1)
	{
		HandleCentered();
	}
	else if (m_left_edge == -1)
	{
		HandleLeftSided();
	}
	else // m_right_edge == -1
	{
		HandleRightSided();
	}
}

void TrackAnalyzer::Reset()
{
	m_prev_mid = CCD_MID_POS;
	m_mid = CCD_MID_POS;
}

bool TrackAnalyzer::IsFill(const bitset<LinearCcd::SENSOR_W> &ccd_data)
{
	m_is_all_black = true;
	m_is_all_white = true;
	for (int i = VALID_OFFSET; i < VALID_PIXEL - VALID_OFFSET
			&& (m_is_all_black || m_is_all_white); ++i)
	{
		if (ccd_data[i])
		{
			m_is_all_black = false;
		}
		else
		{
			m_is_all_white = false;
		}
	}
	return (m_is_all_black || m_is_all_white);
}

void TrackAnalyzer::DetectEdge(const bitset<LinearCcd::SENSOR_W> &ccd_data)
{
	m_left_edge = -1;
	for (int i = libutil::ClampVal<int>(VALID_OFFSET, m_prev_mid,
			VALID_PIXEL + VALID_OFFSET - 1); i >= VALID_OFFSET; --i)
	{
		if (!ccd_data[i])
		{
			m_left_edge = i;
			break;
		}
	}

	m_right_edge = -1;
	for (int i = libutil::ClampVal<int>(VALID_OFFSET, m_prev_mid,
			VALID_PIXEL + VALID_OFFSET - 1); i < VALID_PIXEL + VALID_OFFSET; ++i)
	{
		if (!ccd_data[i])
		{
			m_right_edge = i;
			break;
		}
	}
}

void TrackAnalyzer::HandleCentered()
{
	m_mid = (m_left_edge + m_right_edge) / 2;
}

void TrackAnalyzer::HandleLeftSided()
{
	const int track_w = m_edge * 2;
	const int detect_w = m_right_edge - VALID_OFFSET;
	const int imaginery_w = std::max<int>(track_w - detect_w, 0);
	const int imaginery_left_edge = VALID_OFFSET - imaginery_w;
	m_mid = (imaginery_left_edge + m_right_edge) / 2;
}

void TrackAnalyzer::HandleRightSided()
{
	const int track_w = m_edge * 2;
	const int detect_w = VALID_PIXEL - m_left_edge;
	const int imaginery_w = std::max<int>(track_w - detect_w, 0);
	const int imaginery_right_edge = VALID_PIXEL + imaginery_w;
	m_mid = (m_left_edge + imaginery_right_edge) / 2;
}

}

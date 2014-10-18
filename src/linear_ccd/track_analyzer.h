/*
 * track_analyzer.h
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_TRACK_ANALYZER_H_
#define LINEAR_CCD_TRACK_ANALYZER_H_

#include <bitset>

#include <libsc/k60/linear_ccd.h>

namespace linear_ccd
{

class TrackAnalyzer
{
public:
	TrackAnalyzer(const int mid, const int edge);

	void Analyze(const std::bitset<libsc::k60::LinearCcd::kSensorW> &ccd_data);
	void Reset();

	void SetMid(const int mid)
	{
		m_mid = mid;
	}

	void SetEdge(const int edge)
	{
		m_edge = edge;
	}

	int GetPrevMid() const
	{
		return m_prev_mid;
	}

	int GetCurrMid() const
	{
		return m_curr_mid;
	}

	int GetLeftEdge() const
	{
		return m_left_edge;
	}

	int GetRightEdge() const
	{
		return m_right_edge;
	}

	bool IsAllBlack() const
	{
		return m_is_all_black;
	}

	bool IsAllWhite() const
	{
		return m_is_all_white;
	}

private:
	bool IsFill(const std::bitset<libsc::k60::LinearCcd::kSensorW> &ccd_data);
	void DetectEdge(const std::bitset<libsc::k60::LinearCcd::kSensorW> &ccd_data);

	void HandleCentered();
	void HandleLeftSided();
	void HandleRightSided();

	int m_mid;
	int m_edge;
	int m_prev_mid;
	int m_curr_mid;
	int m_left_edge;
	int m_right_edge;
	bool m_is_all_black;
	bool m_is_all_white;
};

}

#endif /* LINEAR_CCD_TRACK_ANALYZER_H_ */

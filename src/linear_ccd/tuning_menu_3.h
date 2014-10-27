/*
 * program_tuning_3.h
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_TUNING_MENU_3_H_
#define LINEAR_CCD_TUNING_MENU_3_H_

#include <cstdint>

#include <libutil/remote_var_manager.h>

namespace linear_ccd
{

class Car;

}

namespace linear_ccd
{

class TuningMenu3
{
public:
	explicit TuningMenu3(Car *const car);

	void Run();

	uint32_t GetCcdThreshold() const
	{
		return m_ccd_threshold->GetInt();
	}

	uint32_t GetEdge() const
	{
		return m_edge->GetInt();
	}

	float GetTurnKp() const
	{
		return m_turn_kp->GetReal();
	}

	float GetTurnKd() const
	{
		return m_turn_kd->GetReal();
	}

	float GetTurnTurnKp() const
	{
		return m_turn_turn_kp->GetReal();
	}

	float GetTurnTurnKd() const
	{
		return m_turn_turn_kd->GetReal();
	}

	uint32_t GetSpeedSp() const
	{
		return m_speed_sp->GetInt();
	}

	float GetSpeedKp() const
	{
		return m_speed_kp->GetReal();
	}

	float GetSpeedKi() const
	{
		return m_speed_ki->GetReal();
	}

	float GetSpeedKd() const
	{
		return m_speed_kd->GetReal();
	}

	uint32_t GetSpeedTurnSp() const
	{
		return m_speed_turn_sp->GetInt();
	}

private:
	enum Page
	{
		TURN,
		SPEED,

		SIZE,
	};

	static constexpr int TUNABLE_INT_COUNT = 11;

	void Select(const int id);
	void SwitchPage(const int id);

	int GetMultiplier() const;

	void AdjustValueTurn(const bool is_positive);
	void AdjustValueSpeed(const bool is_positive);

	void Redraw(const bool is_clear_screen);

	Car *m_car;

	Page m_page;
	int m_select;

	libutil::RemoteVarManager::Var *m_ccd_threshold;
	libutil::RemoteVarManager::Var *m_edge;
	libutil::RemoteVarManager::Var *m_turn_kp;
	libutil::RemoteVarManager::Var *m_turn_kd;
	libutil::RemoteVarManager::Var *m_turn_turn_kp;
	libutil::RemoteVarManager::Var *m_turn_turn_kd;

	libutil::RemoteVarManager::Var *m_speed_sp;
	libutil::RemoteVarManager::Var *m_speed_kp;
	libutil::RemoteVarManager::Var *m_speed_ki;
	libutil::RemoteVarManager::Var *m_speed_kd;
	libutil::RemoteVarManager::Var *m_speed_turn_sp;
};

}

#endif /* LINEAR_CCD_TUNING_MENU_3_H_ */

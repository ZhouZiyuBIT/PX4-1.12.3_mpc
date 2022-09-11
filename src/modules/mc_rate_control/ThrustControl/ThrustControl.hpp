/**
 * @file ThrustControl.hpp
 *
 * thrust PID controller
 */

#pragma once

#include <matrix/matrix/math.hpp>

#include <lib/mixer/MultirotorMixer/MultirotorMixer.hpp>
#include <uORB/topics/rate_ctrl_status.h>
#include <lib/ecl/geo/geo.h>

class ThrustControl
{
public:
	ThrustControl() = default;
	~ThrustControl() = default;

	void setThrustAccelLimit(const float &thrust_min, const float & thrust_max){_thrust_min = thrust_min; _thrust_max = thrust_max;}

	/**
	 * Set the thrust control gains
	 * @param P proportional gain
	 * @param I integral gain
	 * @param D derivative gain
	 */
	void setGains(const float &P, const float &I, const float &D);

	/**
	 * Set the mximum absolute value of the integrator
	 * @param integrator_limit limit value, eg: 1.f
	 */
	void setIntegratorLimit(const float &integrator_limit) { _lim_int = integrator_limit; };

	/**
	 * Set direct thrust to actuator feed forward gain
	 * @see _gain_ff
	 * @param FF feed forward gains
	 */
	void setFeedForwardGain(const float &FF) { _gain_ff = FF; };

	/**
	 * Run one control loop cycle calculation
	 * @param accel_z estimation of body acceleration z
	 * @param norm_thrust_sp desired vehicle body normalized acceleration z setpoint
	 * @param dt
	 * @return [0,1] normalized output to apply to the vehicle
	 */
	float update(const float &accel_z, const float &norm_thrust_sp, const float dt, const bool landed);

	/**
	 * Set the integral term to 0 to prevent windup
	 * @see __int
	 */
	void resetIntegral() { __int = 0; }

private:
	void updateIntegral(float &accel_err, const float dt);

	// Gains
	float _gain_p; ///< thrust control proportional gain
	float _gain_i; ///< thrust control integral gain
	float _gain_d; ///< thrust control derivative gain
	float _lim_int; ///< integrator term maximum absolute value
	float _gain_ff; ///< direct thrust to actuator feed forward gain
	float _thrust_min = 0;
	float _thrust_max = 0;

	// States
	float __int; ///< integral term of the thrust controller
};

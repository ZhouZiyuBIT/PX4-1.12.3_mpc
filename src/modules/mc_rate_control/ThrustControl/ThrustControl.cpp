/**
 * @file ThrustControl.cpp
 */

#include <ThrustControl.hpp>
#include <px4_platform_common/defines.h>

using namespace matrix;

void ThrustControl::setGains(const float &P, const float &I, const float &D)
{
	_gain_p = P;
	_gain_i = I;
	_gain_d = D;
}


float ThrustControl::update(const float &accel_z, const float &norm_thrust_sp, const float dt, const bool landed)
{
	float accel_z_sp = -( (1-norm_thrust_sp)*_thrust_min + norm_thrust_sp*_thrust_max );

	float accel_z_err = accel_z - accel_z_sp;

	// PID control with feed forward
	float out = _gain_p*accel_z_err + __int - _gain_ff*accel_z_sp;

	// update integral only if we are not landed
	if (!landed) {
		updateIntegral(accel_z_err, dt);
	}

	out = math::constrain(out, 0.f, 1.f);

	return out;
}

void ThrustControl::updateIntegral(float &accel_err, const float dt)
{
	float i_factor = accel_err/CONSTANTS_ONE_G;
	i_factor = math::max(0.0f, 1.f - i_factor*i_factor);

	float thrust_i = __int + i_factor*_gain_i*accel_err*dt;
	if(PX4_ISFINITE(thrust_i))
	{
		__int = math::constrain(thrust_i, -_lim_int, _lim_int);
	}
}

/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file RLSIdentification.cpp
 */

#include <RLSIdentification.hpp>


void RLSIdentification::initialize(const float (&x_init)[3], const float (&x_confidence)[3],
				   const float (&R_diag)[5], const VehicleParameters &params)
{
	_w_lpf.setAll(0.f);
	_q.setAll(0.f);
	_q(0) = 1.f;

	_mass = params.mass;
	_num_rotors = math::constrain(params.num_rotors, 0, RLS_MAX_ROTORS);
	_n_groups = math::constrain(params.n_groups, 1, RLS_MAX_GROUPS);
	_lpf_motor_tau = params.lpf_motor_tau;

	for (int i = 0; i < RLS_MAX_ROTORS; i++) {
		_position[i] = params.position[i];
		_axis[i] = params.axis[i];
		_thrust_coef[i] = params.thrust_coef[i];
		_moment_ratio[i] = params.moment_ratio[i];
		_motor_group[i] = params.motor_group[i];
	}

	//RLS Thrust
	_force_vector.setAll(0.f);
	_K_thrust.setAll(0.f);
	_H_thrust.setAll(0.f);
	_prediction_error_thrust.setAll(0.f);

	// One thrust constant per group, all initialised to the same guess/confidence.
	// Unused groups (g >= _n_groups) receive no rotor contribution, so their H
	// columns stay zero and they never update.
	SquareMatrix<float, RLS_MAX_GROUPS> P_init_thrust;
	P_init_thrust.setAll(0.f);
	_xp_thrust.setAll(0.f);

	for (int g = 0; g < RLS_MAX_GROUPS; g++) {
		_xp_thrust(g) = x_init[0];
		P_init_thrust(g, g) = x_confidence[0];
	}

	_Pp_thrust = P_init_thrust;

	SquareMatrix<float, 3> R_thrust;
	R_thrust.setIdentity();
	R_thrust(0, 0) = R_diag[0];
	R_thrust(1, 1) = R_diag[1];
	R_thrust(2, 2) = R_diag[2];
	_R_thrust = R_thrust;

	//RLS offset
	_moment_vector.setAll(0.f);
	_K_offset.setAll(0.f);
	_H_offset.setAll(0.f);
	_prediction_error_offset.setAll(0.f);

	SquareMatrix<float, 3> P_init_offset;
	P_init_offset.setIdentity();
	P_init_offset(0, 0) = x_confidence[1];
	P_init_offset(1, 1) = x_confidence[2];
	P_init_offset(2, 2) = 0.f;  // Results for z offset not accurate, thus setting to 0
	_Pp_offset = P_init_offset;

	SquareMatrix<float, 3> R_offset;
	R_offset.setIdentity();
	R_offset(0, 0) = R_diag[3];
	R_offset(1, 1) = R_diag[4];
	R_offset(2, 2) = R_diag[4];
	_R_offset = R_offset;

	_xp_offset(0) = x_init[1];
	_xp_offset(1) = x_init[2];
	_xp_offset(2) = 0.f;  // Results for z offset not accurate, thus setting to 0
}

void RLSIdentification::updateThrust(const Vector3f &y, const Vector<float, RLS_MAX_ROTORS> &speeds, const float &dt,
				     const bool &interaction_flag, const bool &apply_lpf)
{
	//x[k] = x[k-1] + K[k]*(y[k]-H[k]*x[k-1])
	_dt = dt;

	if (apply_lpf) {
		_updateLpf(speeds);

	} else {
		_w_lpf = speeds;
	}

	_createHThrust();
	_computeKThrust(interaction_flag);

	_computePredictionErrorThrust(y * _mass);
	Vector<float, RLS_MAX_GROUPS> x_thrust = _xp_thrust + _K_thrust * _prediction_error_thrust;

	_computePThrust();
	_xp_thrust = x_thrust;
}

void RLSIdentification::updateOffset(const Quatf &q, const bool &interaction_flag)
{
	//x[k] = x[k-1] + K[k]*(y[k]-H[k]*x[k-1])
	_q = q;
	_createHOffset();
	_computeKOffset(interaction_flag);

	_createMomentVector();
	_computePredictionErrorOffset();
	Vector3f x_offset = _xp_offset + (_K_offset * _prediction_error_offset);

	_computePOffset();
	_xp_offset = x_offset;
}

inline void RLSIdentification::_updateLpf(const Vector<float, RLS_MAX_ROTORS> &u)
{
	//y[k] = (alpha*u[k] + (1-alpha)*y[k-1])
	// alpha = dt/(tau + dt)
	const float alpha = _dt / (_lpf_motor_tau + _dt);
	_w_lpf = alpha * u + (1.f - alpha) * _w_lpf ;
}

inline void RLSIdentification::_createHThrust()
{
	// Actuator force in body frame = sum_r k_f[group(r)] * CT_r * w_r^2 * axis_r.
	// Each column g of H accumulates the (dForce/dk_f_group) contribution of the
	// rotors belonging to group g. Columns for unused groups stay zero.
	_H_thrust.setAll(0.f);

	for (int r = 0; r < _num_rotors; r++) {
		const int g = _motor_group[r];

		if (g < 0 || g >= _n_groups) {
			continue;
		}

		const float w2 = _w_lpf(r) * _w_lpf(r);
		const Vector3f col = _kf_multiplier * _thrust_coef[r] * w2 * _axis[r];

		_H_thrust(0, g) += col(0);
		_H_thrust(1, g) += col(1);
		_H_thrust(2, g) += col(2);
	}
}

inline void RLSIdentification::_computeKThrust(const bool &interaction_flag)
{
	//  K[k] = P[k-1]*H[k]'*inv(H[k]*P[k-1]*H[k]'+R[k])
	if (!interaction_flag) {
		SquareMatrix<float, 3> Q;
		Q = (_H_thrust * _Pp_thrust * _H_thrust.transpose()) + _R_thrust;
		_K_thrust = _Pp_thrust * _H_thrust.transpose() * inv(Q);

	} else {
		_K_thrust.setAll(0.f);
	}
}

inline void RLSIdentification::_computePThrust()
{
	//  P[k] = (I - K[k]H[k])*P[k-1]
	SquareMatrix<float, RLS_MAX_GROUPS> I;
	I.setIdentity();
	_Pp_thrust = (I - _K_thrust * _H_thrust) * _Pp_thrust;
}

inline void RLSIdentification::_computePredictionErrorThrust(const Vector3f &y)
{
	//e[k] = (y[k]-H[k]*x[k-1])
	_prediction_error_thrust = y - (_H_thrust * _xp_thrust);
}

inline void RLSIdentification::_createHOffset()
{
	Vector3f Rz = _q.dcm_z();
	SquareMatrix<float, 3> H;
	H.setAll(0.f);

	H(0, 0) = 0.f;
	H(0, 1) = -Rz(2);
	H(0, 2) = Rz(1);

	H(1, 0) = Rz(2);
	H(1, 1) = 0.f;
	H(1, 2) = -Rz(0);

	H(2, 0) = -Rz(1);
	H(2, 1) = Rz(0);
	H(2, 2) = 0.f;

	_H_offset = _mass * _GRAVITY * H;
}

inline void RLSIdentification::_computeKOffset(const bool &interaction_flag)
{
	//  K[k] = P[k-1]*H[k]'*inv(H[k]*P[k-1]*H[k]'+R[k])
	if (!interaction_flag) {
		SquareMatrix<float, 3> Q;
		Q = (_H_offset * _Pp_offset * _H_offset.transpose()) + _R_offset;
		_K_offset = _Pp_offset * _H_offset.transpose() * inv(Q);

	} else {
		_K_offset.setAll(0.f);
	}
}

inline void RLSIdentification::_computePredictionErrorOffset()
{
	//e[k] = (y[k]-H[k]*x[k-1])
	_prediction_error_offset = _moment_vector - (_H_offset * _xp_offset);
}

inline void RLSIdentification::_computePOffset()
{
	//  P[k] = (I - K[k]H[k])*P[k-1]
	SquareMatrix<float, 3> I;
	I.setIdentity();
	_Pp_offset = (I - _K_offset * _H_offset) * _Pp_offset;
}

inline void RLSIdentification::_createMomentVector()
{
	// Actuator force and moment in body frame, using the same convention as the
	// control allocator (ActuatorEffectivenessRotors):
	//   thrust_r = ct_r * axis_r
	//   moment_r = ct_r * (position_r x axis_r) - ct_r * km_r * axis_r
	// scaled by the estimated k_f of the rotor's group and by w_r^2.
	Vector3f Ft(0.f, 0.f, 0.f);
	Vector3f Qi(0.f, 0.f, 0.f);

	for (int r = 0; r < _num_rotors; r++) {
		const int g = _motor_group[r];
		const float kf = (g >= 0 && g < _n_groups) ? _xp_thrust(g) * _kf_multiplier : 0.f;
		const float w2 = _w_lpf(r) * _w_lpf(r);

		// scalar control-force magnitude of this rotor
		const float u = kf * _thrust_coef[r] * w2;

		const Vector3f Fi = u * _axis[r];
		Ft += Fi;
		Qi += _position[r].cross(Fi) - (_moment_ratio[r] * u) * _axis[r];
	}

	_moment_vector = Qi;
	_force_vector = Ft;
}

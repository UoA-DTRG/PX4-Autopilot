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
 * @file RLSIdentification.hpp
 *
 * PMEN - Identify rotor thrust constants and CoM offset.
 *
 * RLS has the following structure:
 * y = Hx + v
 * where y is an m-element noisy measurement vector
 * x is a constant but unknown parameter n-vector and v is measurement noise
 * H is m x n regressor matrix (depends on the rotor geometry)
 * RLS estimator can be written as
 * x[k] = x[k-1] + K[k]*(y[k]-H[k]*x[k-1])
 * K[k] is nxm and the estimator gain matrix
 * K[k] = P[k-1]*H[k]'*inv(H[k]*P[k-1]*H[k]'+R[k])
 * P[k] = (I - K[k]H[k])*P[k-1]
 * R is mxm the measurement covariance matrix
 * P is nxn the estimator error covariance
 *
 *  For computing the motor thrust constants:
 *  x is a vector of one thrust constant k_f per group [k_f_0 ... k_f_(G-1)]*1e-6
 *  y is a function of acceleration + external forces corrupted by noise
 *  H is a function of the rotor speeds (w^2) and the per-rotor geometry
 *
 *  The rotor geometry (position, thrust axis, thrust coefficient CT and
 *  moment ratio KM) is taken from the control allocator (CA_ROTOR* params).
 *  Rotors are assigned to thrust-coefficient groups; each group shares one
 *  estimated k_f. A coaxial/bottom prop therefore goes in its own group whose
 *  k_f absorbs its reduced effectiveness (no separate reduction coefficient).
 *
 * R[0] should be a parameter based on accelerometer noise
 * x[0] should be a parameter with initial guess
 * P[0] should be a parameter based on confidence of the initial guess
 * Possibly, K needs to be set to 0 during interaction and landed state
 *
 * Inputs: acceleration [body frame], rotor speeds [rad/s]
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

using namespace matrix;

/** Maximum number of rotors (matches control allocator NUM_ROTORS_MAX). */
static constexpr int RLS_MAX_ROTORS = 12;
/** Maximum number of thrust-coefficient groups (one per rotor at most). */
static constexpr int RLS_MAX_GROUPS = 12;

struct VehicleParameters {
	float mass;
	int num_rotors;
	int n_groups;
	float lpf_motor_tau;
	Vector3f position[RLS_MAX_ROTORS];    //< rotor position in body frame [m]
	Vector3f axis[RLS_MAX_ROTORS];        //< rotor thrust axis (unit) in body frame
	float thrust_coef[RLS_MAX_ROTORS];    //< per-rotor relative thrust coefficient (CT)
	float moment_ratio[RLS_MAX_ROTORS];   //< per-rotor moment/thrust ratio (KM, sign = spin)
	int motor_group[RLS_MAX_ROTORS];      //< thrust-coefficient group index of each rotor
};

class RLSIdentification
{
public:

	RLSIdentification() = default;
	~RLSIdentification() = default;

	void initialize(const float (&x_init)[3], const float (&x_confidence)[3],
			const float (&R_diag)[5], const VehicleParameters &params);

	/**
	 * @param y      body-frame acceleration [m/s^2]
	 * @param speeds per-rotor speed [rad/s]
	 * @param dt     sampling time [s]
	 * @param interaction_flag  freeze the RLS gains during interaction
	 * @param apply_lpf  filter the speeds through the first-order motor model
	 *                   (PWM-derived speeds); set false when speeds are measured
	 *                   directly (ESC RPM).
	 */
	void updateThrust(const Vector3f &y, const Vector<float, RLS_MAX_ROTORS> &speeds, const float &dt,
			  const bool &interaction_flag, const bool &apply_lpf);
	void updateOffset(const Quatf &q, const bool &interaction_flag);

	Vector<float, RLS_MAX_ROTORS> getFilteredSpeeds() const { return _w_lpf; }

	Vector3f getPredictionErrorThrust() const { return _prediction_error_thrust; }
	Vector<float, RLS_MAX_GROUPS> getEstimationThrust() const { return _xp_thrust; }
	Vector3f getActuatorForceVector() const { return _force_vector; }
	int getNumGroups() const { return _n_groups; }

	Vector3f getPredictionErrorOffset() const { return _prediction_error_offset; }
	Vector3f getEstimationOffset() const { return _xp_offset; }
	Vector3f getActuatorMomentVector() const { return _moment_vector; }

private:

	void _updateLpf(const Vector<float, RLS_MAX_ROTORS> &u);

	//RLS Thrust
	void _createHThrust();
	void _computeKThrust(const bool &interaction_flag);
	void _computePredictionErrorThrust(const Vector3f &y);
	void _computePThrust();

	//RLS offset
	void _createHOffset();
	void _computeKOffset(const bool &interaction_flag);
	void _computePredictionErrorOffset();
	void _computePOffset();
	void _createMomentVector();

	float _mass{0.8f}; //< Vehicle Mass
	int _num_rotors{4}; //< Number of rotors
	int _n_groups{1}; //< Number of thrust-coefficient groups
	float _dt{0.004f}; //< Sampling Time
	float _lpf_motor_tau{0.1f};  //< Motor Dynamics Time Constant

	// Per-rotor geometry (from the control allocator)
	Vector3f _position[RLS_MAX_ROTORS]{};
	Vector3f _axis[RLS_MAX_ROTORS]{};
	float _thrust_coef[RLS_MAX_ROTORS]{};
	float _moment_ratio[RLS_MAX_ROTORS]{};
	int _motor_group[RLS_MAX_ROTORS]{};

	Vector<float, RLS_MAX_ROTORS> _w_lpf{};
	Quatf _q{};  //Quaternion rotation from the FRD body frame to the NED earth frame

	//RLS Thrust
	Vector<float, RLS_MAX_GROUPS> _xp_thrust{};
	Matrix<float, RLS_MAX_GROUPS, 3> _K_thrust{};
	Matrix<float, 3, RLS_MAX_GROUPS> _H_thrust{};
	SquareMatrix<float, RLS_MAX_GROUPS> _Pp_thrust{};
	SquareMatrix<float, 3> _R_thrust{};
	Vector3f _prediction_error_thrust{};

	//RLS offset
	Vector3f _xp_offset{};
	SquareMatrix<float, 3> _K_offset{};
	SquareMatrix<float, 3> _H_offset{};
	SquareMatrix<float, 3> _Pp_offset{};
	SquareMatrix<float, 3> _R_offset{};
	Vector3f _prediction_error_offset{};
	Vector3f _moment_vector{};
	Vector3f _force_vector{};

	static constexpr float _GRAVITY = 9.80665f; // m/s^2
	static constexpr float _kf_multiplier = (1E-6f);
};

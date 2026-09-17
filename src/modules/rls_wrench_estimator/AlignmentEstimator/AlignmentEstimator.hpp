/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file AlignmentEstimator.hpp
 *
 * Separates a sensor-to-rotor-frame misalignment from a steady external force.
 *
 * A small misalignment eps between the frame the IMU reports in and the frame the
 * rotor geometry is defined in leaves a residual
 *
 *     r = m*a - F_model = eps x F
 *
 * With F dominated by its z component this is
 *
 *     r_x =  Fz * eps_y
 *     r_y = -Fz * eps_x
 *
 * which spans exactly the same two degrees of freedom as a steady external
 * lateral force. At a fixed heading the two are therefore indistinguishable: no
 * estimator can separate them, because the information is not in the data.
 *
 * They do behave differently under rotation - the misalignment term is fixed in
 * the body frame, an aerodynamic or contact force is fixed in the world frame -
 * so this estimator carries both and is observable only while the vehicle yaws:
 *
 *     [r_x]   [  0    Fz   R00  R10 ] [eps_x]
 *     [r_y] = [-Fz     0   R01  R11 ] [eps_y]
 *                                     [w_N  ]
 *                                     [w_E  ]
 *
 * where R rotates body -> NED. Heading coverage is tracked so a caller can refuse
 * an estimate that was taken at a single heading, where the split between the two
 * is decided by the priors rather than by the measurements.
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

using namespace matrix;

class AlignmentEstimator
{
public:
	AlignmentEstimator() { reset(); }
	~AlignmentEstimator() = default;

	/** Number of headings sampled; coverage is reported in whole bins. */
	static constexpr int N_BINS = 12;
	/** Heading span below which the misalignment/force split is not identifiable. */
	static constexpr float MIN_COVERAGE_DEG = 90.f;
	/** Heading span below which the split is identifiable but poorly conditioned. */
	static constexpr float GOOD_COVERAGE_DEG = 180.f;

	void reset();

	/**
	 * @param residual  measured minus modelled body force, i.e. the raw thrust
	 *                  prediction error [N]
	 * @param force     modelled actuator force, body frame [N]
	 * @param q         attitude, rotates body -> NED
	 */
	void update(const Vector3f &residual, const Vector3f &force, const Quatf &q);

	/** Misalignment rotation vector (x, y) [rad]: r = eps x F */
	Vector2f getMisalignment() const { return Vector2f(_x(0), _x(1)); }

	/**
	 * Correction to apply to RLS_EST_ALN_R / RLS_EST_ALN_P [rad].
	 * This is an increment on whatever alignment was already applied when the
	 * residuals were produced, not an absolute value.
	 */
	Vector2f getAlignmentCorrection() const { return Vector2f(-_x(0), -_x(1)); }

	/** Steady world-fixed force separated out from the misalignment, NED [N] */
	Vector2f getExternalForce() const { return Vector2f(_x(2), _x(3)); }

	/** 1-sigma uncertainty of the misalignment states [rad] */
	float getMisalignmentStdDev() const;

	/** Span of headings visited, in degrees */
	float getCoverageDeg() const;

	/** True once the vehicle has turned far enough for the split to be identifiable */
	bool hasEnoughRotation() const { return getCoverageDeg() >= MIN_COVERAGE_DEG; }

	/** True when the rotation gate has passed and the estimate has converged */
	bool isValid() const;

	int getSampleCount() const { return _samples; }

private:
	// Priors: a few degrees of misalignment, a couple of newtons of external force.
	// At a single heading these decide how the residual is shared between the two,
	// which is precisely why isValid() also requires rotation.
	static constexpr float P0_MISALIGNMENT = 0.0076f;   // (5 deg)^2 [rad^2]
	static constexpr float P0_FORCE = 4.0f;             // (2 N)^2 [N^2]
	static constexpr float R_MEAS = 0.09f;              // (0.3 N)^2 [N^2]
	static constexpr float CONVERGED_STDDEV = 0.0087f;  // 0.5 deg [rad]

	Vector<float, 4> _x{};
	SquareMatrix<float, 4> _P{};
	bool _bin_seen[N_BINS] {};
	int _samples{0};
};

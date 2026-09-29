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
 * @file AlignmentEstimator.cpp
 */

#include <AlignmentEstimator.hpp>

// Out-of-line definitions: pre-C++17 these are still required wherever the
// constants are odr-used, such as being bound to a reference by a test macro.
constexpr int AlignmentEstimator::N_BINS;
constexpr float AlignmentEstimator::MIN_COVERAGE_DEG;
constexpr float AlignmentEstimator::GOOD_COVERAGE_DEG;

void AlignmentEstimator::reset()
{
	_x.setAll(0.f);

	_P.setAll(0.f);
	_P(0, 0) = P0_MISALIGNMENT;
	_P(1, 1) = P0_MISALIGNMENT;
	_P(2, 2) = P0_FORCE;
	_P(3, 3) = P0_FORCE;

	for (int i = 0; i < N_BINS; i++) {
		_bin_seen[i] = false;
	}

	_samples = 0;
}

void AlignmentEstimator::update(const Vector3f &residual, const Vector3f &force, const Quatf &q)
{
	if (!residual.isAllFinite() || !force.isAllFinite() || !q.isAllFinite()) {
		return;
	}

	const float fz = force(2);

	// Without vertical thrust there is no lever for the misalignment to act through,
	// so the regressor collapses and the update carries no information.
	if (fabsf(fz) < 1.f) {
		return;
	}

	const Dcmf R(q); // body -> NED

	Matrix<float, 2, 4> H;
	H.setAll(0.f);
	H(0, 1) = fz;
	H(0, 2) = R(0, 0);
	H(0, 3) = R(1, 0);
	H(1, 0) = -fz;
	H(1, 2) = R(0, 1);
	H(1, 3) = R(1, 1);

	Vector2f y(residual(0), residual(1));

	// K = P*H'*inv(H*P*H' + R); P*H' is shared with the innovation covariance.
	const Matrix<float, 4, 2> PHt = _P * H.transpose();
	SquareMatrix<float, 2> S = H * PHt;
	S(0, 0) += R_MEAS;
	S(1, 1) += R_MEAS;

	const float det = S(0, 0) * S(1, 1) - S(0, 1) * S(1, 0);

	if (fabsf(det) < 1e-9f) {
		return;
	}

	SquareMatrix<float, 2> S_inv;
	S_inv(0, 0) =  S(1, 1) / det;
	S_inv(0, 1) = -S(0, 1) / det;
	S_inv(1, 0) = -S(1, 0) / det;
	S_inv(1, 1) =  S(0, 0) / det;

	const Matrix<float, 4, 2> K = PHt * S_inv;

	_x += K * (y - H * _x);
	_P -= K * (H * _P);

	// Keep the covariance symmetric; the asymmetric form drifts over long runs.
	_P = (_P + _P.transpose()) * 0.5f;

	// Heading coverage. Binning rather than a min/max span so that it is immune to
	// wrapping and does not credit a vehicle for oscillating about one heading.
	const float yaw = Eulerf(q).psi();
	int bin = (int)floorf((yaw + M_PI_F) / (2.f * M_PI_F) * (float)N_BINS);
	bin = math::constrain(bin, 0, N_BINS - 1);
	_bin_seen[bin] = true;

	_samples++;
}

float AlignmentEstimator::getMisalignmentStdDev() const
{
	return sqrtf(math::max(math::max(_P(0, 0), _P(1, 1)), 0.f));
}

float AlignmentEstimator::getCoverageDeg() const
{
	int seen = 0;

	for (int i = 0; i < N_BINS; i++) {
		if (_bin_seen[i]) {
			seen++;
		}
	}

	return (float)seen * (360.f / (float)N_BINS);
}

bool AlignmentEstimator::isValid() const
{
	return hasEnoughRotation() && (getMisalignmentStdDev() < CONVERGED_STDDEV) && (_samples > 100);
}

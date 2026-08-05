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
 * @file ControlAllocationPseudoInverse.hpp
 *
 * Simple Control Allocation Algorithm
 *
 * It computes the pseudo-inverse of the effectiveness matrix
 * Actuator saturation is handled by simple clipping, do not
 * expect good performance in case of actuator saturation.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#pragma once

#include "ControlAllocation.hpp"

#include <px4_platform_common/module_params.h>
#include <uORB/topics/parameter_update.h>


class ControlAllocationPseudoInverse: public ControlAllocation, public ModuleParams
{
public:
	ControlAllocationPseudoInverse() : ModuleParams(nullptr) {};
	virtual ~ControlAllocationPseudoInverse() = default;

	void allocate() override;
	void setEffectivenessMatrix(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness,
				    const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point, int num_actuators,
				    bool update_normalization_scale) override;
	void setMetricAllocation(bool metric_allocation) { _metric_allocation = metric_allocation; }

	bool getMixer(matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> &mixer) final;

	void updateParameters() override { updateParams(); }

	/** Number of motors that can be assigned to a vertical-thrust scaling group (matches the MIX0_EDIT_MOTORS bitmask) */
	static constexpr int MAX_VAR_MIXER_MOTORS = 12;
protected:
	matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> _mix;

	bool _mix_update_needed{false};
	bool _metric_allocation{false};

	/**
	 * Recalculate pseudo inverse if required.
	 *
	 */
	void updatePseudoInverse();

	void updateParams() override { ModuleParams::updateParams(); }

private:
	bool readMixerFromCSV(const char *filename,
			      matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> &mixer);

	void normalizeControlAllocationMatrix();
	void updateControlAllocationMatrixScale();

	/**
	 * Re-distribute the vertical thrust (THRUST_Z) column of the mixer between motor groups.
	 *
	 * The motors selected by MIX0_EDIT_MOTORS get a THRUST_Z gain of MIX0_EDIT_VAL times the gain of the
	 * first motor outside the group. The column is then rescaled so that the total vertical authority
	 * (and therefore the hover throttle and the z-loop gain) is unchanged. Only THRUST_Z is touched,
	 * so roll/pitch/yaw allocation is left as computed by the pseudo-inverse.
	 */
	void applyThrustZGroupScaling();

	bool _normalization_needs_update{false};

	DEFINE_PARAMETERS_CUSTOM_PARENT(
		ModuleParams,
		(ParamInt<px4::params::DTRG_MIXER_CSV>) _csv_mixer,
		(ParamInt<px4::params::DTRG_MIXER_NORM>) _mixer_normalization,
		(ParamFloat<px4::params::MIX0_EDIT_VAL>) _param_mixer_edit_val,
		(ParamInt<px4::params::MIX0_EDIT_MOTORS>) _param_mixer_edit_motors,
		(ParamBool<px4::params::MIX_EDIT_EN>) _param_mixer_edit_en
	);
};

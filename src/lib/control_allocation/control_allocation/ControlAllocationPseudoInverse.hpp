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

	/**
	 * DTRG CSV mixer: read the mixing matrix from a CSV file, one row per actuator and
	 * one column per control axis (roll, pitch, yaw, thrust x, y, z). Public and static
	 * so the parser can be tested on its own (DtrgMixerCsvTest.cpp). Blank lines are
	 * skipped, an empty cell reads as 0, and cells beyond the last axis are ignored.
	 *
	 * Lines may be of any length.
	 *
	 * @param result if not null, why the file was rejected (or LOADED), the line of the
	 *        error and the number of rows read
	 * @return false if the file cannot be opened, holds no rows, has a row with fewer
	 *         cells than axes or a cell that is not a finite number (including one
	 *         longer than 31 characters), in which case @p mixer is left untouched
	 */
	static bool readMixerFromCSV(const char *filename,
				     matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> &mixer,
				     CsvMixerResult *result = nullptr);

	CsvMixerResult getCsvMixerResult() const override { return _csv_mixer_result; }

protected:
	matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> _mix;

	const char *_csv_mixer_path{"/fs/microsd/etc/mixer.csv"};

	bool _mix_update_needed{false};
	bool _metric_allocation{false};

	/**
	 * Recalculate pseudo inverse if required.
	 *
	 */
	void updatePseudoInverse();

	void updateParams() override { ModuleParams::updateParams(); }

private:

	void normalizeControlAllocationMatrix();
	void updateControlAllocationMatrixScale();
	bool _normalization_needs_update{false};

	/**
	 * DTRG CSV mixer: (re)read the mixer file and set _mix. A rejected file (see
	 * CsvMixerStatus) leaves _mix all zero, so that commander refuses to arm, unless a
	 * valid file was loaded before: then that mixer is kept, so that a failed re-read
	 * while flying (the file is re-read whenever the effectiveness is updated, e.g. on a
	 * parameter change) does not cut the motors.
	 */
	void loadCsvMixer();

	CsvMixerResult _csv_mixer_result{};
	matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> _csv_mix_last_valid; ///< before normalization
	bool _csv_mix_valid{false};

	DEFINE_PARAMETERS_CUSTOM_PARENT(
		ModuleParams,
		(ParamInt<px4::params::DTRG_MIXER_CSV>) _csv_mixer,
		(ParamInt<px4::params::DTRG_MIXER_NORM>) _mixer_normalization
	);
};

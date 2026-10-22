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

#include "dtrgMixerCheck.hpp"

#include <stdio.h>

DtrgMixerChecks::DtrgMixerChecks()
{
	// the parameter is absent from builds without control_allocator
	_param_csv_mixer = param_find_no_notification("DTRG_MIXER_CSV");
}

void DtrgMixerChecks::checkAndReport(const Context &context, Report &reporter)
{
	if (context.isArmed() || (_param_csv_mixer == PARAM_INVALID)) {
		return;
	}

	// control_allocator reads DTRG_MIXER_CSV at boot only, so what it reports is what it
	// flies with: a rejected file blocks arming even if the parameter has been set back
	// to 0 since. The parameter only tells an enabled but not yet loaded mixer apart.
	int32_t csv_mixer = 0;
	param_get(_param_csv_mixer, &csv_mixer);

	dtrg_mixer_status_s status{};

	if (!_dtrg_mixer_status_sub.copy(&status)) {
		status.status = dtrg_mixer_status_s::STATUS_DISABLED;
	}

	if ((status.status == dtrg_mixer_status_s::STATUS_DISABLED) && (csv_mixer == 0)) {
		return;
	}

	// Short text for mavlink_log. Keep "Preflight Fail: " + text + "\t" inside
	// MAVLINK_LOG_MAXLEN (50) or it is truncated.
	char text[33];

	switch (status.status) {
	case dtrg_mixer_status_s::STATUS_LOADED:
		return;

	case dtrg_mixer_status_s::STATUS_DISABLED:
		/* EVENT
		 * @description
		 * <param>DTRG_MIXER_CSV</param> is enabled, but control_allocator has not loaded a mixer
		 * file and still uses the geometry. The parameter is only read at boot: reboot after
		 * enabling it.
		 */
		reporter.armingCheckFailure(NavModes::All, health_component_t::system,
					    events::ID("check_dtrg_mixer_not_loaded"),
					    events::Log::Error, "DTRG CSV mixer not loaded, reboot");
		snprintf(text, sizeof(text), "DTRG mixer not loaded, reboot");
		break;

	case dtrg_mixer_status_s::STATUS_FILE_NOT_FOUND:
		/* EVENT
		 * @description
		 * The mixer file /fs/microsd/etc/mixer.csv cannot be opened: the SD card is missing or
		 * the file does not exist. The mixer is all zero, so the motors would not respond.
		 *
		 * <profile name="dev">
		 * Copy the mixer to the SD card, or set <param>DTRG_MIXER_CSV</param> to 0 to use the geometry.
		 * </profile>
		 */
		reporter.armingCheckFailure(NavModes::All, health_component_t::system,
					    events::ID("check_dtrg_mixer_file_not_found"),
					    events::Log::Error, "DTRG CSV mixer file not found");
		snprintf(text, sizeof(text), "DTRG mixer file not found");
		break;

	case dtrg_mixer_status_s::STATUS_EMPTY:
		/* EVENT
		 * @description
		 * The mixer file holds no rows. It needs one row per actuator, each with 6 values
		 * (roll, pitch, yaw, thrust x, y, z).
		 */
		reporter.armingCheckFailure(NavModes::All, health_component_t::system,
					    events::ID("check_dtrg_mixer_empty"),
					    events::Log::Error, "DTRG CSV mixer file is empty");
		snprintf(text, sizeof(text), "DTRG mixer file is empty");
		break;

	case dtrg_mixer_status_s::STATUS_SHORT_ROW:
		/* EVENT
		 * @description
		 * Every row of the mixer file needs 6 values (roll, pitch, yaw, thrust x, y, z).
		 * An empty cell between two commas counts as 0.
		 */
		reporter.armingCheckFailure<uint16_t>(NavModes::All, health_component_t::system,
						      events::ID("check_dtrg_mixer_short_row"),
						      events::Log::Error, "DTRG CSV mixer: line {1} has fewer than 6 values", status.line);
		snprintf(text, sizeof(text), "DTRG mixer line %u: < 6 values", (unsigned)status.line);
		break;

	case dtrg_mixer_status_s::STATUS_INVALID_VALUE:
		/* EVENT
		 * @description
		 * A value in the mixer file is not a number: text (e.g. a header row), nan, inf, or a
		 * value longer than 31 characters.
		 */
		reporter.armingCheckFailure<uint16_t>(NavModes::All, health_component_t::system,
						      events::ID("check_dtrg_mixer_invalid_value"),
						      events::Log::Error, "DTRG CSV mixer: line {1} has a value that is not a number", status.line);
		snprintf(text, sizeof(text), "DTRG mixer line %u: bad value", (unsigned)status.line);
		break;

	case dtrg_mixer_status_s::STATUS_ROW_COUNT_MISMATCH:
		/* EVENT
		 * @description
		 * The mixer file needs one row per configured actuator (e.g. <param>CA_ROTOR_COUNT</param>).
		 * Rows beyond the 16th are not read.
		 */
		reporter.armingCheckFailure<uint8_t, uint8_t>(NavModes::All, health_component_t::system,
				events::ID("check_dtrg_mixer_row_count"),
				events::Log::Error, "DTRG CSV mixer has {1} rows, {2} actuators are configured",
				status.num_rows, status.num_actuators);
		snprintf(text, sizeof(text), "DTRG mixer %u rows, %u actuators", (unsigned)status.num_rows,
			 (unsigned)status.num_actuators);
		break;

	case dtrg_mixer_status_s::STATUS_ALL_ZERO:
		/* EVENT
		 * @description
		 * Every value in the mixer file is 0, so the motors would not respond.
		 */
		reporter.armingCheckFailure(NavModes::All, health_component_t::system,
					    events::ID("check_dtrg_mixer_all_zero"),
					    events::Log::Error, "DTRG CSV mixer is all zeros");
		snprintf(text, sizeof(text), "DTRG mixer is all zeros");
		break;

	default:
		/* EVENT
		 * @description
		 * control_allocator reported a mixer status this version of commander does not know.
		 */
		reporter.armingCheckFailure<uint8_t>(NavModes::All, health_component_t::system,
						     events::ID("check_dtrg_mixer_unknown"),
						     events::Log::Error, "DTRG CSV mixer invalid (status {1})", status.status);
		snprintf(text, sizeof(text), "DTRG mixer invalid (%u)", (unsigned)status.status);
		break;
	}

	if (context.isArmingRequest()) {
		// The report above is rate limited and not repeated while it is unchanged, so on its
		// own an arm attempt only shows "Resolve system health failures first". Say why here,
		// without a trailing tab so that ground stations with events support show it too.
		if (!reporter.mavlink_log_pub()) {
			mavlink_log_critical(&_mavlink_log_pub, "Arming denied: %s", text);
		}

	} else if (reporter.mavlink_log_pub()) {
		mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: %s\t", text);
	}
}

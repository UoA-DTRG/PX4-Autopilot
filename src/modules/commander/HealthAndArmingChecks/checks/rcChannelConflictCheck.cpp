/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "rcChannelConflictCheck.hpp"

#include <uORB/topics/input_rc.h>

#include <string.h>

// C++14 still requires out-of-line definitions for odr-used constexpr static members
constexpr const char *RcChannelConflictChecks::kChannelParamPrefix;
constexpr const char *RcChannelConflictChecks::kIgnoredParams[];
constexpr RcChannelConflictChecks::ParamException RcChannelConflictChecks::kExceptions[];

/// bitmask of all channels that can physically exist, bit 0 = channel 1
static constexpr uint32_t kAllChannelsMask = (1u << input_rc_s::RC_INPUT_MAX_CHANNELS) - 1;

RcChannelConflictChecks::RcChannelConflictChecks()
{
	discoverAssignments();
	updateChannelCache();
}

bool RcChannelConflictChecks::isChannelParamName(const char *name)
{
	return strncmp(name, kChannelParamPrefix, strlen(kChannelParamPrefix)) == 0;
}

void RcChannelConflictChecks::discoverAssignments()
{
	// the exceptions go first, so that a parameter matching neither naming rule is
	// still checked, and so that its gate and kind are applied
	for (const ParamException &exception : kExceptions) {
		addAssignment(exception.name);
	}

	for (unsigned i = 0; i < param_count(); ++i) {
		const param_t handle = param_for_index(i);

		if ((handle == PARAM_INVALID) || (param_type(handle) != PARAM_TYPE_INT32)) {
			continue;
		}

		const char *name = param_name(handle);

		if ((name == nullptr) || !isChannelParamName(name)) {
			continue;
		}

		bool ignored = false;

		for (const char *ignored_name : kIgnoredParams) {
			if (strcmp(name, ignored_name) == 0) {
				ignored = true;
				break;
			}
		}

		if (!ignored) {
			addAssignment(name);
		}
	}
}

void RcChannelConflictChecks::addAssignment(const char *name)
{
	const param_t handle = param_find_no_notification(name);

	if (handle == PARAM_INVALID) {
		// parameter is not part of this build (the owning module is not enabled)
		return;
	}

	for (int i = 0; i < _num_assignments; ++i) {
		if (_assignments[i].handle == handle) {
			return;
		}
	}

	if (_num_assignments >= kMaxAssignments) {
		PX4_ERR("too many RC channel assignment params, %s not checked", name);
		return;
	}

	ChannelAssignment &assignment = _assignments[_num_assignments++];
	assignment.name = param_name(handle);
	assignment.handle = handle;

	for (const ParamException &exception : kExceptions) {
		if (strcmp(assignment.name, exception.name) != 0) {
			continue;
		}

		assignment.kind = exception.kind;

		if (exception.gate_name != nullptr) {
			// a missing gate parameter leaves the handle invalid, which disables the assignment
			assignment.gate_handle = param_find_no_notification(exception.gate_name);
			assignment.gated = true;
			assignment.gate_active_when_zero = exception.gate_active_when_zero;
		}

		break;
	}
}

void RcChannelConflictChecks::updateChannelCache()
{
	for (int i = 0; i < _num_assignments; ++i) {
		_assignments[i].channels = activeChannels(_assignments[i]);
	}
}

void RcChannelConflictChecks::updateParams()
{
	HealthAndArmingCheckBase::updateParams();
	updateChannelCache();
	announceConflicts();
}

uint32_t RcChannelConflictChecks::conflictingChannels() const
{
	uint32_t seen = 0;
	uint32_t conflicting = 0;

	for (int i = 0; i < _num_assignments; ++i) {
		conflicting |= seen & _assignments[i].channels;
		seen |= _assignments[i].channels;
	}

	return conflicting;
}

void RcChannelConflictChecks::announceConflicts()
{
	const uint32_t conflicts = conflictingChannels();
	const hrt_abstime now = hrt_absolute_time();

	if (conflicts == 0) {
		if (_reported_conflicts != 0) {
			mavlink_log_info(&_mavlink_log_pub, "RC channel conflict resolved");
			_reported_conflicts = 0;
		}

		return;
	}

	// Repeat while the conflict stands rather than only on the edge: the first
	// message is often emitted before the ground station has connected, and a
	// statustext is not replayed to a station that joins later.
	if ((conflicts == _reported_conflicts) && (now < _last_conflict_message + kConflictMessageInterval)) {
		return;
	}

	_reported_conflicts = conflicts;
	_last_conflict_message = now;

	// no trailing tab: unlike the arming check report this message has no matching
	// event, so the ground station must show the text itself. Keep it inside
	// MAVLINK_LOG_MAXLEN (50) or it is truncated.
	const bool blocks_arming = (_param_com_arm_rc_conf.get() == 0);
	int reported_pairs = 0;

	for (int i = 0; (i < _num_assignments) && (reported_pairs < kMaxReportedPairs); ++i) {
		for (int j = i + 1; (j < _num_assignments) && (reported_pairs < kMaxReportedPairs); ++j) {
			const uint32_t overlap = _assignments[i].channels & _assignments[j].channels;

			if (overlap == 0) {
				continue;
			}

			const int channel = __builtin_ctz(overlap) + 1;
			++reported_pairs;

			if (blocks_arming) {
				mavlink_log_critical(&_mavlink_log_pub, "RC ch %d: %s and %s", channel,
						     _assignments[i].name, _assignments[j].name);

			} else {
				mavlink_log_warning(&_mavlink_log_pub, "RC ch %d: %s and %s", channel,
						    _assignments[i].name, _assignments[j].name);
			}
		}
	}
}

uint32_t RcChannelConflictChecks::activeChannels(const ChannelAssignment &assignment) const
{
	if (assignment.gated) {
		if (assignment.gate_handle == PARAM_INVALID) {
			return 0;
		}

		int32_t gate = 0;

		if (param_get(assignment.gate_handle, &gate) != PX4_OK) {
			return 0;
		}

		if ((gate == 0) != assignment.gate_active_when_zero) {
			return 0;
		}
	}

	int32_t value = 0;

	if (param_get(assignment.handle, &value) != PX4_OK) {
		return 0;
	}

	// 0 means unassigned; a value outside the RC range is rejected by the consumers
	// anyway, so it cannot collide with anything.
	if (value <= 0) {
		return 0;
	}

	if (assignment.kind == ParamKind::Bitmask) {
		return (uint32_t)value & kAllChannelsMask;
	}

	if (value > input_rc_s::RC_INPUT_MAX_CHANNELS) {
		return 0;
	}

	return 1u << (value - 1);
}

void RcChannelConflictChecks::checkAndReport(const Context &context, Report &reporter)
{
	if (context.isArmed()) {
		// configuration check: the assignments cannot change meaningfully while armed
		return;
	}

	// also from here, not just on parameter update, so that a ground station which
	// connects after the assignment was made still gets told
	announceConflicts();

	// COM_ARM_RC_CONF = 1 keeps the report but leaves arming possible
	const bool blocks_arming = (_param_com_arm_rc_conf.get() == 0);
	const NavModes affected_modes = blocks_arming ? NavModes::All : NavModes::None;

	for (int i = 0; i < _num_assignments; ++i) {
		const uint32_t channels = _assignments[i].channels;

		if (channels == 0) {
			continue;
		}

		for (int j = i + 1; j < _num_assignments; ++j) {
			const uint32_t overlap = channels & _assignments[j].channels;

			if (overlap == 0) {
				continue;
			}

			// report the lowest channel the two have in common
			const int32_t channel = __builtin_ctz(overlap) + 1;

			/* EVENT
			 * @description
			 * Two features are configured to read the same RC channel, so one stick
			 * or switch would drive both at once.
			 *
			 * <profile name="dev">
			 * Assign a different channel to one of them, or set <param>COM_ARM_RC_CONF</param>
			 * to warn without blocking arming.
			 * </profile>
			 */
			reporter.armingCheckFailure<uint8_t>(affected_modes, health_component_t::remote_control,
							     events::ID("check_rc_channel_conflict"),
							     events::Log::Error, "RC channel {1} assigned to more than one function", channel);

			if (blocks_arming && reporter.mavlink_log_pub()) {
				mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: %s and %s both use RC channel %d\t",
						     _assignments[i].name, _assignments[j].name, (int)channel);
			}
		}
	}
}

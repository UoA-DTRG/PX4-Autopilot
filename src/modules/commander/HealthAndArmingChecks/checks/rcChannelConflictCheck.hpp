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

#pragma once

#include "../Common.hpp"

#include <parameters/param.h>

/**
 * Detects two switch or channel functions being assigned to the same raw RC channel.
 *
 * The standard PX4 switches (arm, kill, flight mode, ...) and the DTRG horizontal
 * thrust and bench test features each let the operator nominate an RC channel by
 * number. Nothing stops two of them naming the same channel, in which case one
 * stick or switch drives two functions at once - moving the bench test direction
 * switch would also command a tilt, or flipping the arm switch would kill the
 * motors. This check fails arming while such an overlap is configured.
 *
 * There is no list of parameters to maintain. The parameter table is scanned once
 * at boot and every INT32 parameter whose name starts with "RC_MAP"
 * (kChannelParamPrefix), the PX4 channel mapping convention, is picked up. A new
 * feature therefore joins the check purely by naming its parameter, without
 * touching commander. kExceptions lists the few parameters that need to be treated
 * differently, and kIgnoredParams the ones the convention catches by mistake.
 *
 * Discovery has to be by name at runtime rather than through px4::params::,
 * because some of the parameters are defined by optional modules (bench_test is
 * built into a handful of board configs only) and commander is built for all of
 * them. A parameter that is absent from a build is skipped.
 *
 * The occupied channels are cached and only re-read on a parameter update, so the
 * periodic run of the check does no parameter lookups.
 */
class RcChannelConflictChecks : public HealthAndArmingCheckBase
{
public:
	RcChannelConflictChecks();
	~RcChannelConflictChecks() = default;

	void checkAndReport(const Context &context, Report &reporter) override;

	void updateParams() override;

private:
	/**
	 * How a switch parameter is read.
	 *
	 * Channel parameters hold a 1-based raw channel number with 0 meaning
	 * "unassigned", matching both rc_channels.channels[] (index = value - 1) and
	 * input_rc.values[] (index = value - 1). Bitmask parameters hold one bit per
	 * channel with bit 0 = channel 1, and can therefore occupy several channels.
	 */
	enum class ParamKind {
		Channel,
		Bitmask,
	};

	/// A channel assignment parameter that is not read the way the naming convention implies.
	struct ParamException {
		const char *name;
		const char *gate_name;      ///< parameter deciding whether the assignment is read at all, nullptr if always read
		bool gate_active_when_zero; ///< read the assignment while the gate is zero, instead of while it is non-zero
		ParamKind kind;
	};

	/// Runtime state for one discovered parameter.
	struct ChannelAssignment {
		const char *name{nullptr};
		param_t handle{PARAM_INVALID};
		param_t gate_handle{PARAM_INVALID};
		bool gated{false};
		bool gate_active_when_zero{false};
		ParamKind kind{ParamKind::Channel};
		uint32_t channels{0};       ///< cached result of activeChannels()
	};

	/// Prefix marking a parameter as an RC channel assignment.
	static constexpr const char *kChannelParamPrefix = "RC_MAP";

	/**
	 * The parameters that need more than the naming convention. These are also
	 * checked even when their name does not match kChannelParamPrefix.
	 *
	 * RC_MAP_FLTM_BTN holds a bitmask of up to 6 channels rather than a single
	 * channel number, and is only read while RC_MAP_FLTMODE is unassigned
	 * (rc_update.cpp:577), so a mapping behind an active RC_MAP_FLTMODE cannot
	 * collide with anything.
	 */
	static constexpr ParamException kExceptions[] = {
		{"RC_MAP_FLTM_BTN", "RC_MAP_FLTMODE", true, ParamKind::Bitmask},
	};

	/**
	 * Parameters the naming convention picks up that are not exclusive channel
	 * assignments:
	 * - RC_MAP_MODE_SW is deprecated and no longer read by rc_update, so a value left
	 *   over from an old airframe must not block arming
	 * - RC_MAP_FAILSAFE is documented to sit on the throttle channel (params.c:1207),
	 *   so overlapping with RC_MAP_THROTTLE is the normal configuration
	 */
	static constexpr const char *kIgnoredParams[] = {
		"RC_MAP_MODE_SW",
		"RC_MAP_FAILSAFE",
	};

	static constexpr int kMaxAssignments = 40;

	/// Maximum number of conflicting pairs named in a single burst of messages.
	static constexpr int kMaxReportedPairs = 3;

	/// How often the message is repeated while the same conflict stands.
	static constexpr hrt_abstime kConflictMessageInterval = 30_s;

	ChannelAssignment _assignments[kMaxAssignments];
	int _num_assignments{0};

	/// Channels reported as conflicting last time, to detect a change.
	uint32_t _reported_conflicts{0};
	hrt_abstime _last_conflict_message{0};

	orb_advert_t _mavlink_log_pub{nullptr};

	/// Scan the parameter table for channel assignment parameters. Run once, at construction.
	void discoverAssignments();

	/// @return whether the parameter name follows the channel mapping convention
	static bool isChannelParamName(const char *name);

	/// Add one parameter to _assignments if it exists and is not already known.
	void addAssignment(const char *name);

	/// Re-read all assigned channels into the cache.
	void updateChannelCache();

	/**
	 * @return a bitmask of the raw RC channels this function currently occupies
	 *         (bit 0 = channel 1), or 0 when the feature is disabled, no channel is
	 *         assigned, or the assigned channel is out of range.
	 */
	uint32_t activeChannels(const ChannelAssignment &assignment) const;

	/// @return a bitmask of the raw RC channels claimed by more than one function
	uint32_t conflictingChannels() const;

	/**
	 * Send a plain text message to the ground station while channels conflict, so
	 * that a bad assignment is visible immediately rather than only in the arming
	 * check report. Emitted when the set of conflicting channels changes and then
	 * repeated every kConflictMessageInterval while it stands.
	 */
	void announceConflicts();

	DEFINE_PARAMETERS_CUSTOM_PARENT(HealthAndArmingCheckBase,
					(ParamInt<px4::params::COM_ARM_RC_CONF>) _param_com_arm_rc_conf
				       )
};

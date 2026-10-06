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
 * @file dtrg_horizontal_thrust.hpp
 *
 * DTRG horizontal thrust (HT) logic shared by mc_att_control (Stabilized) and
 * mc_pos_control (Position / Offboard): reading the HT switch and aux tilt
 * channels, and deciding per DTRG_HT_MASK which body axes are driven by
 * horizontal thrust and which by tilting.
 *
 * DTRG_HT_MASK selects the axes moved by horizontal thrust (HT axes):
 * - 0: X and Y
 * - 1: X; Y movement by rolling (normal controller or roll stick)
 * - 2: Y; X movement by pitching (normal controller or pitch stick)
 *
 * DTRG_HT_SPLIT_EN selects how the HT axes move:
 * - 0: by horizontal thrust only. Their tilt comes from the aux tilt channels
 *      or the offboard HT attitude (level when unassigned).
 * - 1: by horizontal thrust and by tilting. DTRG_HT_SPLIT is the share
 *      produced by horizontal thrust, the rest is produced by tilting with the
 *      normal controller or the sticks.
 *
 * Header-only so the unit tests (DtrgHorizontalThrustTest.cpp) can run it
 * without either module.
 */

#pragma once

#include <float.h>
#include <math.h>
#include <stdint.h>

#include <mathlib/math/Limits.hpp>
#include <px4_platform_common/defines.h>
#include <uORB/topics/rc_channels.h>

namespace dtrg_ht
{

/// rc_channels value above which the HT switch (RC_MAP_HT_MODE) reads as on (PWM ~1750 with default calibration)
static constexpr float kSwitchThreshold = 0.5f;

/// aux tilt channel values within +-kAuxDeadzone of centre command no tilt
static constexpr float kAuxDeadzone = 0.02f;

static constexpr int kNumChannels = static_cast<int>(sizeof(rc_channels_s::channels) / sizeof(
		rc_channels_s::channels[0]));

/**
 * Convert a 1-based RC_MAP_HT_* parameter into a 0-based index into rc_channels.channels[].
 * @return the index, or -1 when the parameter is 0 (unassigned) or negative
 */
static inline int channelIndex(int32_t rc_map_param)
{
	return (rc_map_param > 0) ? static_cast<int>(rc_map_param - 1) : -1;
}

/// @return whether @p index can be used on rc_channels.channels[]
static inline bool validChannelIndex(int index)
{
	return (index >= 0) && (index < kNumChannels);
}

/**
 * @param rc           latest rc_channels sample
 * @param enabled      DTRG_HT_EN
 * @param switch_index channelIndex(RC_MAP_HT_MODE)
 * @return whether horizontal thrust is switched on
 */
static inline bool switchActive(const rc_channels_s &rc, bool enabled, int switch_index)
{
	return enabled && validChannelIndex(switch_index) && (rc.channels[switch_index] > kSwitchThreshold);
}

/**
 * Tilt setpoint from an aux tilt channel (RC_MAP_HT_ROLL / RC_MAP_HT_PITCH).
 *
 * @param rc    latest rc_channels sample
 * @param index channelIndex() of the channel parameter, -1 disables the input
 * @param limit tilt magnitude at full deflection [rad] (DTRG_HT_R_MAX / DTRG_HT_P_MAX)
 * @return the channel scaled by @p limit and clamped to +-@p limit, 0 inside the deadzone,
 *         for a disabled channel or for a non-finite value
 */
static inline float auxTiltSetpoint(const rc_channels_s &rc, int index, float limit)
{
	if (!validChannelIndex(index)) {
		return 0.f;
	}

	const float raw = rc.channels[index];

	if (!PX4_ISFINITE(raw) || (fabsf(raw) <= kAuxDeadzone)) {
		return 0.f;
	}

	return math::constrain(raw * limit, -limit, limit);
}

/// highest DTRG_HT_MASK the modules act on
static constexpr int32_t kMaxSelectableMask = 2;

/**
 * DTRG_HT_MASK as used by the modules.
 * @return @p mask_param, or 0 (horizontal thrust X and Y) when it is outside 0..kMaxSelectableMask
 */
static inline int32_t selectableMask(int32_t mask_param)
{
	return ((mask_param >= 0) && (mask_param <= kMaxSelectableMask)) ? mask_param : 0;
}

/// @return whether DTRG_HT_MASK moves along body X with horizontal thrust (masks 0 and 1)
static inline bool maskUsesX(int32_t mask)
{
	return mask != 2;
}

/// @return whether DTRG_HT_MASK moves along body Y with horizontal thrust (masks 0 and 2)
static inline bool maskUsesY(int32_t mask)
{
	return mask != 1;
}

/// DTRG_HT_SPLIT default, also used for a non-finite value
static constexpr float kDefaultSplit = 0.5f;

/**
 * Share of the movement on an HT axis produced by horizontal thrust.
 *
 * @param split_en DTRG_HT_SPLIT_EN
 * @param split    DTRG_HT_SPLIT
 * @return @p split constrained to [0, 1] with the split enabled, otherwise 1 (horizontal thrust only)
 */
static inline float thrustShare(bool split_en, float split)
{
	if (!split_en) {
		return 1.f;
	}

	return PX4_ISFINITE(split) ? math::constrain(split, 0.f, 1.f) : kDefaultSplit;
}

/**
 * Share of the movement on an HT axis produced by tilting with the normal controller or the sticks.
 *
 * @param split_en DTRG_HT_SPLIT_EN
 * @param split    DTRG_HT_SPLIT
 * @return 1 - thrustShare() with the split enabled, otherwise 0 (the tilt comes from the
 *         aux channel or the offboard HT attitude instead)
 */
static inline float tiltShare(bool split_en, float split)
{
	return split_en ? (1.f - thrustShare(split_en, split)) : 0.f;
}

struct HorizontalThrust {
	float x{0.f};
	float y{0.f};
	bool x_sat{false}; ///< x is at the DTRG_HT_MAX limit
	bool y_sat{false}; ///< y is at the DTRG_HT_MAX limit
};

/**
 * Body frame horizontal thrust for the given mask. On an axis the mask moves by tilting
 * only (Y for mask 1, X for mask 2) the horizontal thrust is 0.
 *
 * @param mask     DTRG_HT_MASK
 * @param x_demand demanded body X thrust (normalised)
 * @param y_demand demanded body Y thrust (normalised)
 * @param limit    DTRG_HT_MAX
 */
static inline HorizontalThrust horizontalThrust(int32_t mask, float x_demand, float y_demand, float limit)
{
	HorizontalThrust ht{};

	if (maskUsesX(mask)) {
		ht.x = math::constrain(x_demand, -limit, limit);
	}

	if (maskUsesY(mask)) {
		ht.y = math::constrain(y_demand, -limit, limit);
	}

	ht.x_sat = fabsf(ht.x) >= limit;
	ht.y_sat = fabsf(ht.y) >= limit;
	return ht;
}

struct NorthEast {
	float north{0.f};
	float east{0.f};
};

/**
 * Horizontal NED thrust the position controller tilts for with the split enabled: the
 * thrust along the heading (body X) and across it (body Y) scaled by tiltShare() on an
 * HT axis, unchanged on the other axis. Horizontal thrust produces the rest.
 *
 * @param mask     DTRG_HT_MASK
 * @param split_en DTRG_HT_SPLIT_EN
 * @param split    DTRG_HT_SPLIT
 * @param north    controller thrust north
 * @param east     controller thrust east
 * @param yaw      yaw setpoint [rad]
 */
static inline NorthEast tiltThrust(int32_t mask, bool split_en, float split, float north, float east, float yaw)
{
	const float tilt_share = tiltShare(split_en, split);
	const float x_scale = maskUsesX(mask) ? tilt_share : 1.f;
	const float y_scale = maskUsesY(mask) ? tilt_share : 1.f;

	const float cos_yaw = cosf(yaw);
	const float sin_yaw = sinf(yaw);

	// into the heading frame (x forward, y right), scale, and back to north/east
	const float forward = (cos_yaw * north + sin_yaw * east) * x_scale;
	const float right = (-sin_yaw * north + cos_yaw * east) * y_scale;

	return NorthEast{cos_yaw *forward - sin_yaw * right, sin_yaw *forward + cos_yaw * right};
}

struct Tilt {
	float roll{0.f};
	float pitch{0.f};
};

/**
 * Roll and pitch for the position controller with HT active: the HT tilt (normally level)
 * on an axis moved by horizontal thrust only, the controller's tilt on every other axis.
 *
 * @param mask       DTRG_HT_MASK
 * @param split_en   DTRG_HT_SPLIT_EN; with the split enabled the HT axes tilt with the controller too
 * @param ht_roll    roll from the aux channel, or from the offboard HT attitude
 * @param ht_pitch   pitch from the aux channel, or from the offboard HT attitude
 * @param ctrl_roll  roll the position controller asked for
 * @param ctrl_pitch pitch the position controller asked for
 */
static inline Tilt positionControlTilt(int32_t mask, bool split_en, float ht_roll, float ht_pitch, float ctrl_roll,
				       float ctrl_pitch)
{
	const bool roll_from_ht = maskUsesY(mask) && !split_en;
	const bool pitch_from_ht = maskUsesX(mask) && !split_en;

	return Tilt{roll_from_ht ? ht_roll : ctrl_roll, pitch_from_ht ? ht_pitch : ctrl_pitch};
}

/**
 * Roll and pitch for Stabilized with HT active, before the manual tilt limit and the input
 * filters. On an HT axis the stick tilts for tiltShare() of its tilt with the split enabled,
 * otherwise the aux channel sets the tilt. On the other axis the stick tilts as usual.
 *
 * @param mask        DTRG_HT_MASK
 * @param split_en    DTRG_HT_SPLIT_EN
 * @param split       DTRG_HT_SPLIT
 * @param roll_stick  roll stick * MPC_MAN_TILT_MAX [rad]
 * @param pitch_stick pitch stick * MPC_MAN_TILT_MAX [rad]
 * @param roll_knob   roll from the aux channel [rad]
 * @param pitch_knob  pitch from the aux channel [rad], in the pitch stick's sign convention
 */
static inline Tilt manualModeTilt(int32_t mask, bool split_en, float split, float roll_stick, float pitch_stick,
				  float roll_knob, float pitch_knob)
{
	const float tilt_share = tiltShare(split_en, split);
	const float ht_roll = split_en ? roll_stick * tilt_share : roll_knob;
	const float ht_pitch = split_en ? pitch_stick * tilt_share : pitch_knob;

	return Tilt{maskUsesY(mask) ? ht_roll : roll_stick, maskUsesX(mask) ? ht_pitch : pitch_stick};
}

/**
 * Body frame horizontal thrust for Stabilized: full stick commands DTRG_HT_MAX on the axes
 * selected by the mask, only thrustShare() of it with the split enabled (the tilt produces
 * the rest).
 *
 * @param mask        DTRG_HT_MASK
 * @param split_en    DTRG_HT_SPLIT_EN
 * @param split       DTRG_HT_SPLIT
 * @param roll_stick  roll stick [-1, 1], body Y
 * @param pitch_stick pitch stick [-1, 1], body X
 * @param limit       DTRG_HT_MAX
 */
static inline HorizontalThrust manualModeHorizontalThrust(int32_t mask, bool split_en, float split, float roll_stick,
		float pitch_stick, float limit)
{
	const float ht_max = limit * thrustShare(split_en, split);
	return horizontalThrust(mask, pitch_stick * ht_max, roll_stick * ht_max, limit);
}

} // namespace dtrg_ht

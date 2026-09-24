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
 * DTRG_HT_MASK selects how the vehicle moves along each horizontal axis:
 * - 0: horizontal thrust on both X and Y; the vehicle stays level (roll and
 *      pitch come from the aux tilt channels or the offboard HT attitude).
 * - 1: horizontal thrust on X; Y movement by rolling (normal controller or
 *      roll stick). Pitch comes from the aux/offboard tilt.
 * - 2: horizontal thrust on Y; X movement by pitching (normal controller or
 *      pitch stick). Roll comes from the aux/offboard tilt.
 * - 3: no horizontal thrust; X and Y movement by pitching and rolling, as
 *      without HT.
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

/// @return whether DTRG_HT_MASK moves along body X with horizontal thrust (masks 0 and 1)
static inline bool maskUsesX(int32_t mask)
{
	return (mask != 2) && (mask != 3);
}

/// @return whether DTRG_HT_MASK moves along body Y with horizontal thrust (masks 0 and 2)
static inline bool maskUsesY(int32_t mask)
{
	return (mask != 1) && (mask != 3);
}

struct HorizontalThrust {
	float x{0.f};
	float y{0.f};
	bool x_sat{false}; ///< x is at the DTRG_HT_MAX limit
	bool y_sat{false}; ///< y is at the DTRG_HT_MAX limit
};

/**
 * Body frame horizontal thrust for the given mask. On an axis the mask moves by tilting
 * instead (Y for mask 1, X for mask 2, both for mask 3) the horizontal thrust is 0.
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

struct Tilt {
	float roll{0.f};
	float pitch{0.f};
};

/**
 * Roll and pitch for the position controller with HT active: the controller's tilt on
 * an axis moved by tilting, the HT tilt (normally level) on an axis moved by horizontal thrust.
 *
 * @param mask       DTRG_HT_MASK
 * @param ht_roll    roll from the aux channel, or from the offboard HT attitude
 * @param ht_pitch   pitch from the aux channel, or from the offboard HT attitude
 * @param ctrl_roll  roll the position controller asked for
 * @param ctrl_pitch pitch the position controller asked for
 */
static inline Tilt positionControlTilt(int32_t mask, float ht_roll, float ht_pitch, float ctrl_roll, float ctrl_pitch)
{
	switch (mask) {
	case 1:
		return Tilt{ctrl_roll, ht_pitch};

	case 2:
		return Tilt{ht_roll, ctrl_pitch};

	case 3:
		return Tilt{ctrl_roll, ctrl_pitch};

	default:
		return Tilt{ht_roll, ht_pitch};
	}
}

} // namespace dtrg_ht

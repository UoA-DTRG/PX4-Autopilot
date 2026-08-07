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
 * @file bench_test_switch.h
 *
 * Shared decoding of the bench test 3-position direction switch (BT_SIGN_SW).
 * Header-only so commander can apply the same interpretation for its arming
 * check without depending on the bench_test module being built.
 */

#pragma once

#include <stdint.h>

#include <uORB/topics/input_rc.h>

namespace bench_test
{

// Three-position direction switch: above high -> positive, below low -> negative,
// in between (centre) -> no excitation.
static constexpr uint16_t kSignHighThresholdUs = 1700;
static constexpr uint16_t kSignLowThresholdUs  = 1300;

/**
 * Decode a raw RC pulse width into the excitation direction.
 * @return +1 (switch up), -1 (switch down), 0 (centre or unmapped channel)
 */
static inline float signFromPulse(uint16_t value)
{
	if (value > kSignHighThresholdUs) {
		return 1.f;
	}

	// Unpopulated channels report 0, which must not read as "switch down".
	if ((value < kSignLowThresholdUs) && (value > 0)) {
		return -1.f;
	}

	return 0.f;
}

/**
 * Decode the direction switch from an input_rc sample.
 *
 * @param input_rc raw RC sample
 * @param channel  1-based channel number (BT_SIGN_SW), 0 = disabled
 * @return +1 / -1 / 0. Returns 0 (centre) for a disabled or out-of-range
 *         channel and on RC loss, so both users fail safe to "no excitation".
 */
static inline float signFromInputRc(const input_rc_s &input_rc, int32_t channel)
{
	if (channel <= 0) {
		return 0.f;
	}

	if (input_rc.rc_lost || input_rc.rc_failsafe) {
		return 0.f;
	}

	const int channel_index = channel - 1; // BT_SIGN_SW is 1-based

	if ((channel_index >= input_rc.channel_count)
	    || (channel_index >= (int)input_rc_s::RC_INPUT_MAX_CHANNELS)) {
		return 0.f;
	}

	return signFromPulse(input_rc.values[channel_index]);
}

} // namespace bench_test

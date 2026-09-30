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
 * @file DtrgBenchSwitchTest.cpp
 *
 * Decoding of the bench test 3-position direction switch (RC_MAP_CMD_SIGN).
 * Commander uses the same decoding for its "centre the switch" arming check,
 * so a regression here affects both the excitation and the arming gate.
 */

#include <gtest/gtest.h>

#include "bench_test_switch.h"

using bench_test::signFromInputRc;
using bench_test::signFromPulse;

namespace
{

input_rc_s makeInputRc(int channel_count, uint16_t fill = 1500)
{
	input_rc_s rc{};
	rc.channel_count = channel_count;

	for (int i = 0; i < input_rc_s::RC_INPUT_MAX_CHANNELS; i++) {
		rc.values[i] = fill;
	}

	return rc;
}

} // namespace

TEST(DtrgBenchSwitch, PulseThresholds)
{
	// the thresholds themselves are centre, only strictly beyond them counts
	EXPECT_FLOAT_EQ(signFromPulse(2000), 1.f);
	EXPECT_FLOAT_EQ(signFromPulse(1701), 1.f);
	EXPECT_FLOAT_EQ(signFromPulse(1700), 0.f);
	EXPECT_FLOAT_EQ(signFromPulse(1500), 0.f);
	EXPECT_FLOAT_EQ(signFromPulse(1300), 0.f);
	EXPECT_FLOAT_EQ(signFromPulse(1299), -1.f);
	EXPECT_FLOAT_EQ(signFromPulse(1000), -1.f);
	EXPECT_FLOAT_EQ(signFromPulse(1), -1.f);
}

TEST(DtrgBenchSwitch, ZeroPulseIsCentre)
{
	// an unpopulated channel reports 0 and must not read as "switch down"
	EXPECT_FLOAT_EQ(signFromPulse(0), 0.f);
}

TEST(DtrgBenchSwitch, InputRcUsesOneBasedChannel)
{
	input_rc_s rc = makeInputRc(8);
	rc.values[5] = 2000; // channel 6
	rc.values[6] = 1000; // channel 7

	EXPECT_FLOAT_EQ(signFromInputRc(rc, 6), 1.f);
	EXPECT_FLOAT_EQ(signFromInputRc(rc, 7), -1.f);
	EXPECT_FLOAT_EQ(signFromInputRc(rc, 5), 0.f);
}

TEST(DtrgBenchSwitch, DisabledChannelIsCentre)
{
	input_rc_s rc = makeInputRc(8, 2000);

	EXPECT_FLOAT_EQ(signFromInputRc(rc, 0), 0.f);
	EXPECT_FLOAT_EQ(signFromInputRc(rc, -1), 0.f);
}

TEST(DtrgBenchSwitch, ChannelBeyondChannelCountIsCentre)
{
	input_rc_s rc = makeInputRc(8, 2000);

	EXPECT_FLOAT_EQ(signFromInputRc(rc, 8), 1.f);
	EXPECT_FLOAT_EQ(signFromInputRc(rc, 9), 0.f);
	EXPECT_FLOAT_EQ(signFromInputRc(rc, input_rc_s::RC_INPUT_MAX_CHANNELS + 1), 0.f);
}

TEST(DtrgBenchSwitch, ChannelBeyondArrayIsCentreEvenIfCountClaimsMore)
{
	// a corrupt channel_count must not index past values[]
	input_rc_s rc = makeInputRc(input_rc_s::RC_INPUT_MAX_CHANNELS, 2000);
	rc.channel_count = 255;

	EXPECT_FLOAT_EQ(signFromInputRc(rc, input_rc_s::RC_INPUT_MAX_CHANNELS), 1.f);
	EXPECT_FLOAT_EQ(signFromInputRc(rc, input_rc_s::RC_INPUT_MAX_CHANNELS + 1), 0.f);
}

TEST(DtrgBenchSwitch, RcLossIsCentre)
{
	input_rc_s rc = makeInputRc(8, 2000);
	rc.rc_lost = true;
	EXPECT_FLOAT_EQ(signFromInputRc(rc, 6), 0.f);

	rc = makeInputRc(8, 1000);
	rc.rc_failsafe = true;
	EXPECT_FLOAT_EQ(signFromInputRc(rc, 6), 0.f);
}

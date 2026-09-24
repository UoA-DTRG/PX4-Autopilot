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
 * @file DtrgHorizontalThrustTest.cpp
 *
 * DTRG horizontal thrust helpers: HT switch, aux tilt channels and the
 * DTRG_HT_MASK axis selection used by mc_att_control and mc_pos_control.
 */

#include <gtest/gtest.h>

#include <math.h>

#include "dtrg_horizontal_thrust.hpp"

using namespace dtrg_ht;

namespace
{

rc_channels_s makeRc(float fill = 0.f)
{
	rc_channels_s rc{};

	for (int i = 0; i < kNumChannels; i++) {
		rc.channels[i] = fill;
	}

	return rc;
}

constexpr float kLimit = 0.1745f; // 10 deg

} // namespace

// Channel parameters ----------------------------------------------------------

TEST(DtrgHorizontalThrust, ChannelIndexIsZeroBased)
{
	EXPECT_EQ(channelIndex(1), 0);
	EXPECT_EQ(channelIndex(8), 7);
	EXPECT_EQ(channelIndex(18), 17);
}

TEST(DtrgHorizontalThrust, UnassignedChannelIsInvalid)
{
	EXPECT_EQ(channelIndex(0), -1);
	EXPECT_EQ(channelIndex(-5), -1);
	EXPECT_FALSE(validChannelIndex(channelIndex(0)));
}

TEST(DtrgHorizontalThrust, ChannelIndexRange)
{
	EXPECT_TRUE(validChannelIndex(0));
	EXPECT_TRUE(validChannelIndex(kNumChannels - 1));
	EXPECT_FALSE(validChannelIndex(kNumChannels));
	EXPECT_FALSE(validChannelIndex(-1));
}

// HT switch -------------------------------------------------------------------

TEST(DtrgHorizontalThrust, SwitchOnAboveThreshold)
{
	rc_channels_s rc = makeRc();
	const int idx = channelIndex(8);

	rc.channels[idx] = 1.f;
	EXPECT_TRUE(switchActive(rc, true, idx));

	rc.channels[idx] = 0.51f;
	EXPECT_TRUE(switchActive(rc, true, idx));

	rc.channels[idx] = 0.5f;
	EXPECT_FALSE(switchActive(rc, true, idx));

	rc.channels[idx] = 0.f;
	EXPECT_FALSE(switchActive(rc, true, idx));

	rc.channels[idx] = -1.f;
	EXPECT_FALSE(switchActive(rc, true, idx));
}

TEST(DtrgHorizontalThrust, SwitchNeedsHtEnabled)
{
	const rc_channels_s rc = makeRc(1.f);
	EXPECT_FALSE(switchActive(rc, false, channelIndex(8)));
}

TEST(DtrgHorizontalThrust, SwitchOffWhenUnassigned)
{
	const rc_channels_s rc = makeRc(1.f);
	EXPECT_FALSE(switchActive(rc, true, channelIndex(0)));
	EXPECT_FALSE(switchActive(rc, true, kNumChannels));
}

TEST(DtrgHorizontalThrust, SwitchOffOnNan)
{
	rc_channels_s rc = makeRc();
	rc.channels[7] = NAN;
	EXPECT_FALSE(switchActive(rc, true, 7));
}

// Aux tilt channels -----------------------------------------------------------

TEST(DtrgHorizontalThrust, AuxTiltScalesToLimit)
{
	rc_channels_s rc = makeRc();
	rc.channels[8] = 1.f;
	EXPECT_FLOAT_EQ(auxTiltSetpoint(rc, 8, kLimit), kLimit);

	rc.channels[8] = -0.5f;
	EXPECT_FLOAT_EQ(auxTiltSetpoint(rc, 8, kLimit), -0.5f * kLimit);
}

TEST(DtrgHorizontalThrust, AuxTiltDeadzone)
{
	rc_channels_s rc = makeRc();

	rc.channels[8] = 0.02f;
	EXPECT_FLOAT_EQ(auxTiltSetpoint(rc, 8, kLimit), 0.f);

	rc.channels[8] = -0.02f;
	EXPECT_FLOAT_EQ(auxTiltSetpoint(rc, 8, kLimit), 0.f);

	// just outside the deadzone: no rescaling, the raw value is used
	rc.channels[8] = 0.03f;
	EXPECT_FLOAT_EQ(auxTiltSetpoint(rc, 8, kLimit), 0.03f * kLimit);
}

TEST(DtrgHorizontalThrust, AuxTiltIsClamped)
{
	// a mis-calibrated channel beyond +-1 must not exceed the tilt limit
	rc_channels_s rc = makeRc();
	rc.channels[8] = 1.5f;
	EXPECT_FLOAT_EQ(auxTiltSetpoint(rc, 8, kLimit), kLimit);

	rc.channels[8] = -3.f;
	EXPECT_FLOAT_EQ(auxTiltSetpoint(rc, 8, kLimit), -kLimit);
}

TEST(DtrgHorizontalThrust, AuxTiltDisabledChannel)
{
	const rc_channels_s rc = makeRc(1.f);
	EXPECT_FLOAT_EQ(auxTiltSetpoint(rc, channelIndex(0), kLimit), 0.f);
	EXPECT_FLOAT_EQ(auxTiltSetpoint(rc, kNumChannels, kLimit), 0.f);
}

TEST(DtrgHorizontalThrust, AuxTiltNonFinite)
{
	rc_channels_s rc = makeRc();
	rc.channels[8] = NAN;
	EXPECT_FLOAT_EQ(auxTiltSetpoint(rc, 8, kLimit), 0.f);

	rc.channels[8] = INFINITY;
	EXPECT_FLOAT_EQ(auxTiltSetpoint(rc, 8, kLimit), 0.f);
}

// Horizontal thrust per mask --------------------------------------------------

TEST(DtrgHorizontalThrust, MaskAxes)
{
	EXPECT_TRUE(maskUsesX(0));
	EXPECT_TRUE(maskUsesY(0));

	EXPECT_TRUE(maskUsesX(1));
	EXPECT_FALSE(maskUsesY(1));

	EXPECT_FALSE(maskUsesX(2));
	EXPECT_TRUE(maskUsesY(2));

	EXPECT_FALSE(maskUsesX(3));
	EXPECT_FALSE(maskUsesY(3));
}

TEST(DtrgHorizontalThrust, MaskZeroUsesBothAxes)
{
	const HorizontalThrust ht = horizontalThrust(0, 0.2f, -0.1f, 0.5f);
	EXPECT_FLOAT_EQ(ht.x, 0.2f);
	EXPECT_FLOAT_EQ(ht.y, -0.1f);
	EXPECT_FALSE(ht.x_sat);
	EXPECT_FALSE(ht.y_sat);
}

TEST(DtrgHorizontalThrust, MaskOneIsXOnly)
{
	const HorizontalThrust ht = horizontalThrust(1, 0.2f, -0.1f, 0.5f);
	EXPECT_FLOAT_EQ(ht.x, 0.2f);
	EXPECT_FLOAT_EQ(ht.y, 0.f);
}

TEST(DtrgHorizontalThrust, MaskTwoIsYOnly)
{
	const HorizontalThrust ht = horizontalThrust(2, 0.2f, -0.1f, 0.5f);
	EXPECT_FLOAT_EQ(ht.x, 0.f);
	EXPECT_FLOAT_EQ(ht.y, -0.1f);
}

TEST(DtrgHorizontalThrust, MaskThreeHasNoHorizontalThrust)
{
	// both axes move by tilting: even a saturating demand gives no horizontal force
	const HorizontalThrust ht = horizontalThrust(3, 0.8f, -0.9f, 0.5f);
	EXPECT_FLOAT_EQ(ht.x, 0.f);
	EXPECT_FLOAT_EQ(ht.y, 0.f);
	EXPECT_FALSE(ht.x_sat);
	EXPECT_FALSE(ht.y_sat);
}

TEST(DtrgHorizontalThrust, ThrustIsClampedAndFlaggedSaturated)
{
	const HorizontalThrust ht = horizontalThrust(0, 0.8f, -0.9f, 0.5f);
	EXPECT_FLOAT_EQ(ht.x, 0.5f);
	EXPECT_FLOAT_EQ(ht.y, -0.5f);
	EXPECT_TRUE(ht.x_sat);
	EXPECT_TRUE(ht.y_sat);
}

TEST(DtrgHorizontalThrust, ExactlyAtLimitIsSaturated)
{
	const HorizontalThrust ht = horizontalThrust(0, 0.5f, 0.49f, 0.5f);
	EXPECT_TRUE(ht.x_sat);
	EXPECT_FALSE(ht.y_sat);
}

TEST(DtrgHorizontalThrust, UnusedAxisIsNeverSaturated)
{
	// mask 1 ignores a huge Y demand entirely
	const HorizontalThrust ht = horizontalThrust(1, 0.f, 10.f, 0.5f);
	EXPECT_FLOAT_EQ(ht.y, 0.f);
	EXPECT_FALSE(ht.y_sat);
}

TEST(DtrgHorizontalThrust, StickToThrustAsInStabilized)
{
	// mc_att_control feeds stick * DTRG_HT_MAX: full stick is exactly at the limit
	const float limit = 0.3f;
	const HorizontalThrust ht = horizontalThrust(0, 1.f * limit, -0.5f * limit, limit);
	EXPECT_FLOAT_EQ(ht.x, 0.3f);
	EXPECT_FLOAT_EQ(ht.y, -0.15f);
	EXPECT_TRUE(ht.x_sat);
	EXPECT_FALSE(ht.y_sat);
}

// Tilt selection in the position controller -----------------------------------

TEST(DtrgHorizontalThrust, PositionControlTiltPerMask)
{
	const float ht_roll = 0.1f;
	const float ht_pitch = 0.2f;
	const float ctrl_roll = 0.3f;
	const float ctrl_pitch = 0.4f;

	Tilt t = positionControlTilt(0, ht_roll, ht_pitch, ctrl_roll, ctrl_pitch);
	EXPECT_FLOAT_EQ(t.roll, ht_roll);
	EXPECT_FLOAT_EQ(t.pitch, ht_pitch);

	// X by HT: pitch from HT, roll from the controller
	t = positionControlTilt(1, ht_roll, ht_pitch, ctrl_roll, ctrl_pitch);
	EXPECT_FLOAT_EQ(t.roll, ctrl_roll);
	EXPECT_FLOAT_EQ(t.pitch, ht_pitch);

	// Y by HT: roll from HT, pitch from the controller
	t = positionControlTilt(2, ht_roll, ht_pitch, ctrl_roll, ctrl_pitch);
	EXPECT_FLOAT_EQ(t.roll, ht_roll);
	EXPECT_FLOAT_EQ(t.pitch, ctrl_pitch);

	t = positionControlTilt(3, ht_roll, ht_pitch, ctrl_roll, ctrl_pitch);
	EXPECT_FLOAT_EQ(t.roll, ctrl_roll);
	EXPECT_FLOAT_EQ(t.pitch, ctrl_pitch);
}

TEST(DtrgHorizontalThrust, UnknownMaskBehavesAsFullHt)
{
	// DTRG_HT_MASK is bounded to 0..3, but an out of range value must not tilt with the controller
	const Tilt t = positionControlTilt(7, 0.1f, 0.2f, 0.3f, 0.4f);
	EXPECT_FLOAT_EQ(t.roll, 0.1f);
	EXPECT_FLOAT_EQ(t.pitch, 0.2f);

	const HorizontalThrust ht = horizontalThrust(7, 0.2f, 0.1f, 0.5f);
	EXPECT_FLOAT_EQ(ht.x, 0.2f);
	EXPECT_FLOAT_EQ(ht.y, 0.1f);
}

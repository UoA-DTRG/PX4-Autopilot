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
}

TEST(DtrgHorizontalThrust, SelectableMask)
{
	EXPECT_EQ(selectableMask(0), 0);
	EXPECT_EQ(selectableMask(1), 1);
	EXPECT_EQ(selectableMask(2), 2);

	// out of range, including the old mask 3, falls back to 0
	EXPECT_EQ(selectableMask(3), 0);
	EXPECT_EQ(selectableMask(7), 0);
	EXPECT_EQ(selectableMask(-1), 0);
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

// Split between horizontal thrust and tilt on the HT axes ---------------------

TEST(DtrgHorizontalThrust, SplitShares)
{
	EXPECT_FLOAT_EQ(thrustShare(true, 0.5f), 0.5f);
	EXPECT_FLOAT_EQ(tiltShare(true, 0.5f), 0.5f);

	EXPECT_FLOAT_EQ(thrustShare(true, 0.25f), 0.25f);
	EXPECT_FLOAT_EQ(tiltShare(true, 0.25f), 0.75f);

	// 0: tilt only, 1: horizontal thrust only
	EXPECT_FLOAT_EQ(thrustShare(true, 0.f), 0.f);
	EXPECT_FLOAT_EQ(tiltShare(true, 0.f), 1.f);
	EXPECT_FLOAT_EQ(thrustShare(true, 1.f), 1.f);
	EXPECT_FLOAT_EQ(tiltShare(true, 1.f), 0.f);
}

TEST(DtrgHorizontalThrust, SplitDisabledIsHorizontalThrustOnly)
{
	// the HT axes move by horizontal thrust only, whatever DTRG_HT_SPLIT is
	EXPECT_FLOAT_EQ(thrustShare(false, 0.2f), 1.f);
	EXPECT_FLOAT_EQ(tiltShare(false, 0.2f), 0.f);
}

TEST(DtrgHorizontalThrust, SplitIsClamped)
{
	EXPECT_FLOAT_EQ(thrustShare(true, 1.5f), 1.f);
	EXPECT_FLOAT_EQ(tiltShare(true, 1.5f), 0.f);
	EXPECT_FLOAT_EQ(thrustShare(true, -0.5f), 0.f);
	EXPECT_FLOAT_EQ(tiltShare(true, -0.5f), 1.f);

	EXPECT_FLOAT_EQ(thrustShare(true, NAN), kDefaultSplit);
	EXPECT_FLOAT_EQ(tiltShare(true, NAN), 1.f - kDefaultSplit);
}

TEST(DtrgHorizontalThrust, TiltThrustScalesHtAxesOnly)
{
	// yaw 0: heading X is north, heading Y is east
	NorthEast t = tiltThrust(0, true, 0.25f, 0.2f, -0.4f, 0.f);
	EXPECT_NEAR(t.north, 0.15f, 1e-6f);
	EXPECT_NEAR(t.east, -0.3f, 1e-6f);

	// mask 1: only X is an HT axis, Y keeps the full thrust for rolling
	t = tiltThrust(1, true, 0.25f, 0.2f, -0.4f, 0.f);
	EXPECT_NEAR(t.north, 0.15f, 1e-6f);
	EXPECT_NEAR(t.east, -0.4f, 1e-6f);

	// mask 2: only Y is an HT axis
	t = tiltThrust(2, true, 0.25f, 0.2f, -0.4f, 0.f);
	EXPECT_NEAR(t.north, 0.2f, 1e-6f);
	EXPECT_NEAR(t.east, -0.3f, 1e-6f);
}

TEST(DtrgHorizontalThrust, TiltThrustFollowsHeading)
{
	// yaw 90 deg: heading X is east, heading Y is south
	const float yaw = M_PI_F / 2.f;

	// mask 1 scales the thrust along the heading (east) only
	const NorthEast t = tiltThrust(1, true, 0.5f, 0.2f, 0.4f, yaw);
	EXPECT_NEAR(t.north, 0.2f, 1e-6f);
	EXPECT_NEAR(t.east, 0.2f, 1e-6f);
}

TEST(DtrgHorizontalThrust, TiltThrustSplitZeroIsUnchanged)
{
	// split 0 tilts for all of the thrust, as without horizontal thrust
	const NorthEast t = tiltThrust(0, true, 0.f, 0.2f, -0.4f, 0.7f);
	EXPECT_NEAR(t.north, 0.2f, 1e-6f);
	EXPECT_NEAR(t.east, -0.4f, 1e-6f);
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

TEST(DtrgHorizontalThrust, StickToThrustAsInManualMode)
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

	Tilt t = positionControlTilt(0, false, ht_roll, ht_pitch, ctrl_roll, ctrl_pitch);
	EXPECT_FLOAT_EQ(t.roll, ht_roll);
	EXPECT_FLOAT_EQ(t.pitch, ht_pitch);

	// X by HT: pitch from HT, roll from the controller
	t = positionControlTilt(1, false, ht_roll, ht_pitch, ctrl_roll, ctrl_pitch);
	EXPECT_FLOAT_EQ(t.roll, ctrl_roll);
	EXPECT_FLOAT_EQ(t.pitch, ht_pitch);

	// Y by HT: roll from HT, pitch from the controller
	t = positionControlTilt(2, false, ht_roll, ht_pitch, ctrl_roll, ctrl_pitch);
	EXPECT_FLOAT_EQ(t.roll, ht_roll);
	EXPECT_FLOAT_EQ(t.pitch, ctrl_pitch);
}

TEST(DtrgHorizontalThrust, PositionControlTiltWithSplit)
{
	// with the split the HT axes tilt with the (scaled) controller too, for every mask
	for (int32_t mask = 0; mask <= 2; mask++) {
		const Tilt t = positionControlTilt(mask, true, 0.1f, 0.2f, 0.3f, 0.4f);
		EXPECT_FLOAT_EQ(t.roll, 0.3f);
		EXPECT_FLOAT_EQ(t.pitch, 0.4f);
	}
}

TEST(DtrgHorizontalThrust, UnknownMaskBehavesAsFullHt)
{
	// DTRG_HT_MASK is bounded to 0..2, but an out of range value must not tilt with the controller
	const Tilt t = positionControlTilt(7, false, 0.1f, 0.2f, 0.3f, 0.4f);
	EXPECT_FLOAT_EQ(t.roll, 0.1f);
	EXPECT_FLOAT_EQ(t.pitch, 0.2f);

	const HorizontalThrust ht = horizontalThrust(7, 0.2f, 0.1f, 0.5f);
	EXPECT_FLOAT_EQ(ht.x, 0.2f);
	EXPECT_FLOAT_EQ(ht.y, 0.1f);
}

// Manual: how the sticks divide between horizontal thrust and tilt --------

namespace
{

constexpr float kHtMax = 0.5f; // DTRG_HT_MAX
constexpr float kManTiltMax = 0.6109f; // MPC_MAN_TILT_MAX 35 deg

// asymmetric so a thrust/tilt or X/Y swap cannot pass
constexpr float kSplits[] = {0.f, 0.25f, 0.5f, 0.8f, 1.f};

struct Sticks {
	float roll;
	float pitch;
};

constexpr Sticks kSticks[] = {{0.3f, 0.7f}, {-1.f, 0.4f}, {0.6f, -1.f}};

} // namespace

TEST(DtrgHorizontalThrust, ManualModeWithoutSplitHtAxesAreThrustOnly)
{
	// without the split an HT axis moves by thrust only: full stick is DTRG_HT_MAX and the tilt
	// is the aux channel's, whatever DTRG_HT_SPLIT says; the other axis tilts with the stick
	const float roll_knob = 0.05f;
	const float pitch_knob = -0.08f;

	for (int32_t mask = 0; mask <= kMaxSelectableMask; mask++) {
		for (const Sticks &s : kSticks) {
			SCOPED_TRACE(testing::Message() << "mask " << mask << " roll " << s.roll << " pitch " << s.pitch);

			const HorizontalThrust ht = manualModeHorizontalThrust(mask, false, 0.2f, s.roll, s.pitch, kHtMax);
			const Tilt t = manualModeTilt(mask, false, 0.2f, s.roll * kManTiltMax, s.pitch * kManTiltMax, roll_knob,
						      pitch_knob);

			EXPECT_FLOAT_EQ(ht.x, maskUsesX(mask) ? s.pitch *kHtMax : 0.f);
			EXPECT_FLOAT_EQ(ht.y, maskUsesY(mask) ? s.roll *kHtMax : 0.f);
			EXPECT_FLOAT_EQ(t.pitch, maskUsesX(mask) ? pitch_knob : s.pitch * kManTiltMax);
			EXPECT_FLOAT_EQ(t.roll, maskUsesY(mask) ? roll_knob : s.roll * kManTiltMax);
		}
	}
}

TEST(DtrgHorizontalThrust, ManualModeSplitDividesTheStick)
{
	// with the split an HT axis moves by DTRG_HT_SPLIT of the stick as thrust and the rest as
	// tilt, so the two fractions add up to the stick; the other axis has no thrust and tilts
	// for all of the stick. The aux channels are not used.
	for (int32_t mask = 0; mask <= kMaxSelectableMask; mask++) {
		for (const float split : kSplits) {
			for (const Sticks &s : kSticks) {
				SCOPED_TRACE(testing::Message() << "mask " << mask << " split " << split << " roll " << s.roll
					     << " pitch " << s.pitch);

				const HorizontalThrust ht = manualModeHorizontalThrust(mask, true, split, s.roll, s.pitch, kHtMax);
				const Tilt t = manualModeTilt(mask, true, split, s.roll * kManTiltMax, s.pitch * kManTiltMax, 0.05f,
							      -0.08f);

				const float x_thrust = ht.x / kHtMax;
				const float y_thrust = ht.y / kHtMax;
				const float x_tilt = t.pitch / kManTiltMax;
				const float y_tilt = t.roll / kManTiltMax;

				if (maskUsesX(mask)) {
					EXPECT_NEAR(x_thrust, split * s.pitch, 1e-6f);
					EXPECT_NEAR(x_tilt, (1.f - split) * s.pitch, 1e-6f);

				} else {
					EXPECT_FLOAT_EQ(x_thrust, 0.f);
					EXPECT_FLOAT_EQ(x_tilt, s.pitch);
				}

				if (maskUsesY(mask)) {
					EXPECT_NEAR(y_thrust, split * s.roll, 1e-6f);
					EXPECT_NEAR(y_tilt, (1.f - split) * s.roll, 1e-6f);

				} else {
					EXPECT_FLOAT_EQ(y_thrust, 0.f);
					EXPECT_FLOAT_EQ(y_tilt, s.roll);
				}

				EXPECT_NEAR(x_thrust + x_tilt, s.pitch, 1e-6f);
				EXPECT_NEAR(y_thrust + y_tilt, s.roll, 1e-6f);
			}
		}
	}
}

TEST(DtrgHorizontalThrust, ManualModeSplitZeroIsNoHorizontalThrust)
{
	// DTRG_HT_SPLIT_EN=1, DTRG_HT_SPLIT=0 (the old mask 3): standard Manual Mode on every mask
	for (int32_t mask = 0; mask <= kMaxSelectableMask; mask++) {
		const HorizontalThrust ht = manualModeHorizontalThrust(mask, true, 0.f, 1.f, -1.f, kHtMax);
		const Tilt t = manualModeTilt(mask, true, 0.f, 0.3f, -0.4f, 0.05f, -0.08f);
		EXPECT_FLOAT_EQ(ht.x, 0.f);
		EXPECT_FLOAT_EQ(ht.y, 0.f);
		EXPECT_FALSE(ht.x_sat);
		EXPECT_FALSE(ht.y_sat);
		EXPECT_FLOAT_EQ(t.roll, 0.3f);
		EXPECT_FLOAT_EQ(t.pitch, -0.4f);
	}
}

TEST(DtrgHorizontalThrust, ManualModeSplitOneIsLevel)
{
	// split 1 is all thrust: full stick reaches DTRG_HT_MAX and the HT axes stay level,
	// the aux channels are still not used
	const HorizontalThrust ht = manualModeHorizontalThrust(0, true, 1.f, -1.f, 1.f, kHtMax);
	const Tilt t = manualModeTilt(0, true, 1.f, -kManTiltMax, kManTiltMax, 0.05f, -0.08f);
	EXPECT_FLOAT_EQ(ht.x, kHtMax);
	EXPECT_FLOAT_EQ(ht.y, -kHtMax);
	EXPECT_TRUE(ht.x_sat);
	EXPECT_TRUE(ht.y_sat);
	EXPECT_FLOAT_EQ(t.roll, 0.f);
	EXPECT_FLOAT_EQ(t.pitch, 0.f);
}

TEST(DtrgHorizontalThrust, ManualModeSplitNeverSaturates)
{
	// below split 1 full stick asks for split * DTRG_HT_MAX, inside the limit
	const HorizontalThrust ht = manualModeHorizontalThrust(0, true, 0.8f, 1.f, -1.f, kHtMax);
	EXPECT_FLOAT_EQ(ht.x, -0.8f * kHtMax);
	EXPECT_FLOAT_EQ(ht.y, 0.8f * kHtMax);
	EXPECT_FALSE(ht.x_sat);
	EXPECT_FALSE(ht.y_sat);
}

TEST(DtrgHorizontalThrust, ManualModeSplitOutOfRange)
{
	// a split outside 0..1 is clamped and NaN falls back to the default, for thrust and tilt alike
	HorizontalThrust ht = manualModeHorizontalThrust(0, true, 1.5f, 0.f, 1.f, kHtMax);
	Tilt t = manualModeTilt(0, true, 1.5f, 0.f, 0.4f, 0.f, 0.f);
	EXPECT_FLOAT_EQ(ht.x, kHtMax);
	EXPECT_FLOAT_EQ(t.pitch, 0.f);

	ht = manualModeHorizontalThrust(0, true, -0.5f, 0.f, 1.f, kHtMax);
	t = manualModeTilt(0, true, -0.5f, 0.f, 0.4f, 0.f, 0.f);
	EXPECT_FLOAT_EQ(ht.x, 0.f);
	EXPECT_FLOAT_EQ(t.pitch, 0.4f);

	ht = manualModeHorizontalThrust(0, true, NAN, 0.f, 1.f, kHtMax);
	t = manualModeTilt(0, true, NAN, 0.f, 0.4f, 0.f, 0.f);
	EXPECT_FLOAT_EQ(ht.x, kDefaultSplit * kHtMax);
	EXPECT_FLOAT_EQ(t.pitch, (1.f - kDefaultSplit) * 0.4f);
}

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

#pragma once

#include "../Common.hpp"

#include <parameters/param.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/dtrg_mixer_status.h>

/**
 * DTRG CSV mixer (DTRG_MIXER_CSV): refuses to arm unless control_allocator loaded a
 * valid mixer file. A rejected file leaves an all-zero mixer, with which the motors
 * would not respond.
 *
 * DTRG_MIXER_CSV belongs to control_allocator and commander is built for boards
 * without it, so the parameter is looked up by name at runtime; the check does
 * nothing where it is absent.
 */
class DtrgMixerChecks : public HealthAndArmingCheckBase
{
public:
	DtrgMixerChecks();
	~DtrgMixerChecks() = default;

	void checkAndReport(const Context &context, Report &reporter) override;

private:
	param_t _param_csv_mixer{PARAM_INVALID};

	uORB::Subscription _dtrg_mixer_status_sub{ORB_ID(dtrg_mixer_status)};

	orb_advert_t _mavlink_log_pub{nullptr}; ///< for the "Arming denied" message
};

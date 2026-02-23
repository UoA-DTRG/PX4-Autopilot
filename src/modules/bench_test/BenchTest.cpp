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

/**
 * @file BenchTest.cpp
 *
 * Motor bench testing module.
 *
 * Provides CLI commands for exciting individual or all motors with
 * step, impulse, chirp/tweet, and current-sweep profiles while the
 * vehicle remains disarmed. High-rate logging of ESC eRPM (via
 * bidirectional DShot) and power-module voltage/current telemetry
 * is handled by the normal logger – this module only generates the
 * actuator_test commands.
 *
 * Safety:
 *  - Tests are blocked if the vehicle is armed.
 *  - The radio kill-switch (manual_control_switches.kill_switch) is
 *    polled every cycle and will immediately abort any running test.
 *  - Each actuator_test message carries a timeout so the ESC driver
 *    will stop the motor even if this module crashes.
 */

#include "BenchTest.hpp"

#include <inttypes.h>
#include <parameters/param.h>
#include <lib/system_identification/multisine_excitation.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

/* ── Constructor / Destructor ─────────────────────────────────────────── */

BenchTest::BenchTest() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

BenchTest::~BenchTest()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

/* ── Initialisation ───────────────────────────────────────────────────── */

bool BenchTest::init()
{
	ScheduleOnInterval(10_ms); // 100 Hz polling for kill switch & test state machine
	return true;
}

/* ── Scheduled work item callback ─────────────────────────────────────── */

void BenchTest::Run()
{
	if (should_exit()) {
		releaseAllMotors();
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Parameter hot-reload
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	// ── Safety: kill-switch check (RC-based, no MAVLink dependency) ──
	if (_state == TestState::RUNNING && isKillSwitchEngaged()) {
		abortTest("Kill switch engaged – aborting test!");
	}

	// ── Safety: abort if vehicle becomes armed while running ──
	if (_state == TestState::RUNNING && !isDisarmed()) {
		abortTest("Vehicle armed during bench test – aborting!");
	}

	perf_end(_loop_perf);
}

/* ── Motor command helpers ────────────────────────────────────────────── */

void BenchTest::commandMotor(int motor_index, float value, uint32_t timeout_ms)
{
	actuator_test_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.function = actuator_test_s::FUNCTION_MOTOR1 + motor_index;
	cmd.value = value;
	cmd.action = actuator_test_s::ACTION_DO_CONTROL;
	cmd.timeout_ms = timeout_ms;
	_actuator_test_pub.publish(cmd);
}

void BenchTest::releaseAllMotors()
{
	const int n = _param_bt_num_motors.get();

	for (int i = 0; i < n; i++) {
		actuator_test_s cmd{};
		cmd.timestamp = hrt_absolute_time();
		cmd.function = actuator_test_s::FUNCTION_MOTOR1 + i;
		cmd.value = NAN; // NaN → disarmed / stop
		cmd.action = actuator_test_s::ACTION_RELEASE_CONTROL;
		cmd.timeout_ms = 0;
		_actuator_test_pub.publish(cmd);
	}
}

/* ── Safety helpers ───────────────────────────────────────────────────── */

bool BenchTest::isKillSwitchEngaged()
{
	manual_control_switches_s switches;

	if (_manual_switches_sub.copy(&switches)) {
		return switches.kill_switch == manual_control_switches_s::SWITCH_POS_ON;
	}

	return false; // if we can't read, don't spuriously abort
}

bool BenchTest::isDisarmed()
{
	actuator_armed_s armed;

	if (_actuator_armed_sub.copy(&armed)) {
		return !armed.armed;
	}

	return true; // assume disarmed if we can't read
}

void BenchTest::abortTest(const char *reason)
{
	PX4_WARN("%s", reason);
	releaseAllMotors();
	_state = TestState::IDLE;
	_active_test = TestType::NONE;
}

/* ── Test implementations ─────────────────────────────────────────────── */

void BenchTest::runStepTest(int motor)
{
	if (!isDisarmed()) {
		PX4_ERR("Vehicle is armed – refusing to run bench test");
		return;
	}

	if (isKillSwitchEngaged()) {
		PX4_ERR("Kill switch engaged – refusing to start test");
		return;
	}

	const float level     = _param_bt_step_lvl.get();
	const float base      = _param_bt_base_lvl.get();
	const int   hold_ms   = _param_bt_step_dur.get();
	const int   ramp_ms   = _param_bt_ramp_time.get();
	const int   settle_ms = (base > 0.0f) ? _param_bt_bg_setl.get() : 0;
	const int   n         = _param_bt_num_motors.get();
	const int   total_ms  = settle_ms + hold_ms + 2 * ramp_ms + 1000;

	PX4_INFO("Step test: motor %d, base %.2f -> step %.2f, settle %d ms, hold %d ms, ramp %d ms",
		 motor + 1, (double)base, (double)level, settle_ms, hold_ms, ramp_ms);

	_state = TestState::RUNNING;
	_active_test = TestType::STEP_SINGLE;
	_target_motor = motor;
	_test_start_time = hrt_absolute_time();

	// Phase 1: Ramp all motors to base level (if base > 0)
	if (base > 0.0f && ramp_ms > 0) {
		const int ramp_steps = ramp_ms / 10;

		for (int i = 0; i <= ramp_steps && _state == TestState::RUNNING; i++) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during pre-ramp"); return; }

			float frac = (float)i / (float)ramp_steps;

			for (int m = 0; m < n; m++) {
				commandMotor(m, base * frac, total_ms);
			}

			px4_usleep(10000);
		}
	}

	// Phase 2: Hold all motors at base level for settle time
	if (settle_ms > 0) {
		PX4_INFO("  Settling at base %.2f for %d ms...", (double)base, settle_ms);
		const hrt_abstime settle_end = hrt_absolute_time() + (uint64_t)settle_ms * 1000ULL;

		while (hrt_absolute_time() < settle_end && _state == TestState::RUNNING) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during settle"); return; }

			for (int m = 0; m < n; m++) {
				commandMotor(m, base, (uint32_t)((settle_end - hrt_absolute_time()) / 1000 + 500));
			}

			px4_usleep(50000);
		}
	}

	if (_state != TestState::RUNNING) { return; }

	// Phase 3: Ramp target motor from base to step level; all others stay at base
	if (ramp_ms > 0) {
		const int ramp_steps = ramp_ms / 10;

		for (int i = 0; i <= ramp_steps && _state == TestState::RUNNING; i++) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during step ramp-up"); return; }

			float frac = (float)i / (float)ramp_steps;

			for (int m = 0; m < n; m++) {
				float val = (m == motor) ? (base + (level - base) * frac) : base;
				commandMotor(m, val, hold_ms + ramp_ms + 500);
			}

			px4_usleep(10000);
		}
	}

	// Phase 4: Hold target at step level, all others at base
	const hrt_abstime hold_end = hrt_absolute_time() + (uint64_t)hold_ms * 1000ULL;

	while (hrt_absolute_time() < hold_end && _state == TestState::RUNNING) {
		if (isKillSwitchEngaged()) { abortTest("Kill switch during step hold"); return; }

		for (int m = 0; m < n; m++) {
			float val = (m == motor) ? level : base;
			commandMotor(m, val, (uint32_t)((hold_end - hrt_absolute_time()) / 1000 + 500));
		}

		px4_usleep(50000);
	}

	// Phase 5: Ramp target back to base, then ramp all to zero
	if (ramp_ms > 0) {
		const int ramp_steps = ramp_ms / 10;

		for (int i = ramp_steps; i >= 0 && _state == TestState::RUNNING; i--) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during step ramp-down"); return; }

			float frac = (float)i / (float)ramp_steps;

			for (int m = 0; m < n; m++) {
				float val = (m == motor) ? (base + (level - base) * frac) : base;
				commandMotor(m, val, ramp_ms + 500);
			}

			px4_usleep(10000);
		}

		// Ramp all to zero
		for (int i = ramp_steps; i >= 0 && _state == TestState::RUNNING; i--) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during final ramp-down"); return; }

			float frac = (float)i / (float)ramp_steps;

			for (int m = 0; m < n; m++) {
				commandMotor(m, base * frac, ramp_ms + 500);
			}

			px4_usleep(10000);
		}
	}

	releaseAllMotors();
	_state = TestState::IDLE;
	_active_test = TestType::NONE;
	PX4_INFO("Step test complete");
}

void BenchTest::runImpulseTest(int motor)
{
	if (!isDisarmed()) {
		PX4_ERR("Vehicle is armed – refusing to run bench test");
		return;
	}

	if (isKillSwitchEngaged()) {
		PX4_ERR("Kill switch engaged – refusing to start test");
		return;
	}

	const float level     = _param_bt_imp_lvl.get();
	const float base      = _param_bt_base_lvl.get();
	const int   pulse_ms  = _param_bt_imp_dur.get();
	const int   ramp_ms   = _param_bt_ramp_time.get();
	const int   settle_ms = (base > 0.0f) ? _param_bt_bg_setl.get() : 0;
	const int   n         = _param_bt_num_motors.get();

	float impulse_lvl = base + level;
	if (impulse_lvl > 1.0f) { impulse_lvl = 1.0f; }

	PX4_INFO("Impulse test: motor %d, base %.2f, impulse %.2f, settle %d ms, pulse %d ms",
		 motor + 1, (double)base, (double)impulse_lvl, settle_ms, pulse_ms);

	_state = TestState::RUNNING;
	_active_test = TestType::IMPULSE_SINGLE;
	_target_motor = motor;
	_test_start_time = hrt_absolute_time();

	// Phase 1: Ramp all motors to base level (if base > 0)
	if (base > 0.0f && ramp_ms > 0) {
		const int ramp_steps = ramp_ms / 10;

		for (int i = 0; i <= ramp_steps && _state == TestState::RUNNING; i++) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during impulse pre-ramp"); return; }

			float frac = (float)i / (float)ramp_steps;

			for (int m = 0; m < n; m++) {
				commandMotor(m, base * frac, ramp_ms + settle_ms + pulse_ms + 500);
			}

			px4_usleep(10000);
		}
	}

	// Phase 2: Hold all motors at base level for settle time
	if (settle_ms > 0) {
		PX4_INFO("  Settling at base %.2f for %d ms...", (double)base, settle_ms);
		const hrt_abstime settle_end = hrt_absolute_time() + (uint64_t)settle_ms * 1000ULL;

		while (hrt_absolute_time() < settle_end && _state == TestState::RUNNING) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during impulse settle"); return; }

			for (int m = 0; m < n; m++) {
				commandMotor(m, base, (uint32_t)((settle_end - hrt_absolute_time()) / 1000 + 500));
			}

			px4_usleep(50000);
		}
	}

	if (_state != TestState::RUNNING) { return; }

	// Phase 3: Fire impulse on target motor; all others stay at base
	commandMotor(motor, impulse_lvl, pulse_ms + 500);

	const hrt_abstime end = hrt_absolute_time() + (uint64_t)pulse_ms * 1000ULL;

	while (hrt_absolute_time() < end && _state == TestState::RUNNING) {
		if (isKillSwitchEngaged()) { abortTest("Kill switch during impulse"); return; }

		for (int m = 0; m < n; m++) {
			if (m != motor && base > 0.0f) {
				commandMotor(m, base, (uint32_t)((end - hrt_absolute_time()) / 1000 + 500));
			}
		}

		px4_usleep(10000);
	}

	// Phase 4: Ramp all motors back down to zero
	if (base > 0.0f && ramp_ms > 0) {
		const int ramp_steps = ramp_ms / 10;

		for (int i = ramp_steps; i >= 0 && _state == TestState::RUNNING; i--) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during impulse ramp-down"); return; }

			float frac = (float)i / (float)ramp_steps;

			for (int m = 0; m < n; m++) {
				commandMotor(m, base * frac, ramp_ms + 500);
			}

			px4_usleep(10000);
		}
	}

	releaseAllMotors();
	_state = TestState::IDLE;
	_active_test = TestType::NONE;
	PX4_INFO("Impulse test complete");
}

void BenchTest::runTweetTest(int motor)
{
	if (!isDisarmed()) {
		PX4_ERR("Vehicle is armed – refusing to run bench test");
		return;
	}

	if (isKillSwitchEngaged()) {
		PX4_ERR("Kill switch engaged – refusing to start test");
		return;
	}

	const float f_start = _param_bt_twe_f_sta.get();
	const float f_end   = _param_bt_twe_f_end.get();
	const float amp     = _param_bt_twe_amp.get();
	const float bias    = _param_bt_twe_bias.get();
	const int   dur_ms  = _param_bt_twe_dur.get();

	PX4_INFO("Tweet/chirp test: motor %d, freq %.1f-%.1f Hz, amp %.2f, bias %.2f, dur %d ms",
		 motor + 1, (double)f_start, (double)f_end, (double)amp, (double)bias, dur_ms);

	_state = TestState::RUNNING;
	_active_test = TestType::TWEET_SINGLE;
	_target_motor = motor;
	_test_start_time = hrt_absolute_time();

	const float dur_s = (float)dur_ms / 1000.0f;
	const hrt_abstime t0 = hrt_absolute_time();
	const hrt_abstime t_end = t0 + (uint64_t)dur_ms * 1000ULL;

	// Linear chirp:  f(t) = f_start + (f_end - f_start) * t / T
	// Phase:  phi(t) = 2*pi * integral(f(t')) dt' = 2*pi * (f_start*t + 0.5*(f_end-f_start)*t^2/T)
	// Output: bias + amp * sin(phi(t))

	while (hrt_absolute_time() < t_end && _state == TestState::RUNNING) {
		if (isKillSwitchEngaged()) { abortTest("Kill switch during tweet"); return; }

		const float t = (float)(hrt_absolute_time() - t0) / 1e6f; // seconds elapsed
		const float phase = 2.0f * M_PI_F * (f_start * t + 0.5f * (f_end - f_start) * t * t / dur_s);
		float value = bias + amp * sinf(phase);

		// Clamp to [0, 1]
		if (value < 0.0f) { value = 0.0f; }

		if (value > 1.0f) { value = 1.0f; }

		commandMotor(motor, value, 200); // 200 ms timeout as safety
		px4_usleep(2000); // ~500 Hz update rate for smooth chirp
	}

	releaseAllMotors();
	_state = TestState::IDLE;
	_active_test = TestType::NONE;
	PX4_INFO("Tweet/chirp test complete");
}

void BenchTest::runMultiTest()
{
	if (!isDisarmed()) {
		PX4_ERR("Vehicle is armed – refusing to run bench test");
		return;
	}

	if (isKillSwitchEngaged()) {
		PX4_ERR("Kill switch engaged – refusing to start test");
		return;
	}

	const float level = _param_bt_multi_lvl.get();
	const int dur_ms  = _param_bt_multi_dur.get();
	const int ramp_ms = _param_bt_ramp_time.get();
	const int n       = _param_bt_num_motors.get();
	const int total_ms = dur_ms + 2 * ramp_ms;

	PX4_INFO("Multi-motor test: %d motors, level %.2f, dur %d ms, ramp %d ms",
		 n, (double)level, dur_ms, ramp_ms);

	_state = TestState::RUNNING;
	_active_test = TestType::MULTI;
	_test_start_time = hrt_absolute_time();

	const hrt_abstime start = _test_start_time;

	// Ramp up all motors together
	if (ramp_ms > 0) {
		const int ramp_steps = ramp_ms / 10;

		for (int i = 0; i <= ramp_steps && _state == TestState::RUNNING; i++) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during multi ramp-up"); return; }

			float frac = (float)i / (float)ramp_steps;

			for (int m = 0; m < n; m++) {
				commandMotor(m, level * frac, total_ms);
			}

			px4_usleep(10000);
		}
	}

	// Hold
	const hrt_abstime hold_end = start + (uint64_t)(ramp_ms + dur_ms) * 1000ULL;

	while (hrt_absolute_time() < hold_end && _state == TestState::RUNNING) {
		if (isKillSwitchEngaged()) { abortTest("Kill switch during multi hold"); return; }

		for (int m = 0; m < n; m++) {
			commandMotor(m, level, (uint32_t)((hold_end - hrt_absolute_time()) / 1000 + 500));
		}

		px4_usleep(50000);
	}

	// Ramp down
	if (ramp_ms > 0) {
		const int ramp_steps = ramp_ms / 10;

		for (int i = ramp_steps; i >= 0 && _state == TestState::RUNNING; i--) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during multi ramp-down"); return; }

			float frac = (float)i / (float)ramp_steps;

			for (int m = 0; m < n; m++) {
				commandMotor(m, level * frac, ramp_ms + 500);
			}

			px4_usleep(10000);
		}
	}

	releaseAllMotors();
	_state = TestState::IDLE;
	_active_test = TestType::NONE;
	PX4_INFO("Multi-motor test complete");
}

void BenchTest::runCurrentSweep(int motor)
{
	if (!isDisarmed()) {
		PX4_ERR("Vehicle is armed – refusing to run bench test");
		return;
	}

	if (isKillSwitchEngaged()) {
		PX4_ERR("Kill switch engaged – refusing to start test");
		return;
	}

	const float start_lvl = _param_bt_isweep_sta.get();
	const float end_lvl   = _param_bt_isweep_end.get();
	const int   n_steps   = _param_bt_isweep_n.get();
	const int   dwell_ms  = _param_bt_isweep_dwl.get();
	const int   ramp_ms   = _param_bt_ramp_time.get();

	PX4_INFO("Current sweep: motor %d, %.2f -> %.2f in %d steps, dwell %d ms",
		 motor + 1, (double)start_lvl, (double)end_lvl, n_steps, dwell_ms);

	_state = TestState::RUNNING;
	_active_test = TestType::CURRENT_SWEEP_SINGLE;
	_target_motor = motor;
	_test_start_time = hrt_absolute_time();

	float prev_level = 0.0f;

	for (int step = 0; step < n_steps && _state == TestState::RUNNING; step++) {
		const float level = start_lvl + (end_lvl - start_lvl) * (float)step / (float)(n_steps - 1);

		PX4_INFO("  Step %d/%d: throttle %.2f", step + 1, n_steps, (double)level);

		// Ramp from previous level to this level
		if (ramp_ms > 0) {
			const int ramp_steps = ramp_ms / 10;

			for (int i = 0; i <= ramp_steps && _state == TestState::RUNNING; i++) {
				if (isKillSwitchEngaged()) { abortTest("Kill switch during sweep ramp"); return; }

				float frac = (float)i / (float)ramp_steps;
				float val = prev_level + (level - prev_level) * frac;
				commandMotor(motor, val, dwell_ms + ramp_ms + 500);
				px4_usleep(10000);
			}
		}

		// Dwell at this level
		const hrt_abstime dwell_end = hrt_absolute_time() + (uint64_t)dwell_ms * 1000ULL;

		while (hrt_absolute_time() < dwell_end && _state == TestState::RUNNING) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during sweep dwell"); return; }

			commandMotor(motor, level, (uint32_t)((dwell_end - hrt_absolute_time()) / 1000 + 500));
			px4_usleep(50000);
		}

		prev_level = level;
	}

	// Ramp down to zero
	if (ramp_ms > 0 && _state == TestState::RUNNING) {
		const int ramp_steps = ramp_ms / 10;

		for (int i = ramp_steps; i >= 0 && _state == TestState::RUNNING; i--) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during sweep ramp-down"); return; }

			float frac = (float)i / (float)ramp_steps;
			commandMotor(motor, prev_level * frac, ramp_ms + 500);
			px4_usleep(10000);
		}
	}

	releaseAllMotors();
	_state = TestState::IDLE;
	_active_test = TestType::NONE;
	PX4_INFO("Current sweep complete");
}

/* ── Step with background throttle ───────────────────────────────────── */

void BenchTest::runStepWithIdleTest(int motor)
{
	if (!isDisarmed()) {
		PX4_ERR("Vehicle is armed – refusing to run bench test");
		return;
	}

	if (isKillSwitchEngaged()) {
		PX4_ERR("Kill switch engaged – refusing to start test");
		return;
	}

	const float step_lvl  = _param_bt_step_lvl.get();
	const float bg        = _param_bt_bg_lvl.get();
	const int   hold_ms   = _param_bt_step_dur.get();
	const int   ramp_ms   = _param_bt_ramp_time.get();
	const int   settle_ms = _param_bt_bg_setl.get();
	const int   n         = _param_bt_num_motors.get();
	const int   total_ms  = settle_ms + hold_ms + 2 * ramp_ms + 1000;

	PX4_INFO("Step+bg test: motor %d step %.2f, bg %.2f, settle %d ms, hold %d ms, ramp %d ms",
		 motor + 1, (double)step_lvl, (double)bg, settle_ms, hold_ms, ramp_ms);

	_state = TestState::RUNNING;
	_active_test = TestType::STEP_IDLE_BG;
	_target_motor = motor;
	_test_start_time = hrt_absolute_time();

	// Phase 1: Ramp ALL motors (including target) up to BT_BG_LVL together
	PX4_INFO("  Phase 1: Ramp all motors to background level %.2f", (double)bg);

	if (ramp_ms > 0) {
		const int ramp_steps = ramp_ms / 10;

		for (int i = 0; i <= ramp_steps && _state == TestState::RUNNING; i++) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during bg ramp-up"); return; }

			float frac = (float)i / (float)ramp_steps;

			for (int m = 0; m < n; m++) {
				commandMotor(m, bg * frac, total_ms);
			}

			px4_usleep(10000);
		}
	}

	// Phase 2: Hold ALL motors at BT_BG_LVL for settle time
	PX4_INFO("  Phase 2: Settle all motors at bg level for %d ms", settle_ms);
	{
		const hrt_abstime settle_end = hrt_absolute_time() + (uint64_t)settle_ms * 1000ULL;

		while (hrt_absolute_time() < settle_end && _state == TestState::RUNNING) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during bg settle"); return; }

			for (int m = 0; m < n; m++) {
				commandMotor(m, bg, (uint32_t)((settle_end - hrt_absolute_time()) / 1000 + 500));
			}

			px4_usleep(50000);
		}
	}

	if (_state != TestState::RUNNING) { return; }

	// Phase 3: Ramp target motor from BT_BG_LVL up to BT_STEP_LVL; all others stay at BT_BG_LVL
	PX4_INFO("  Phase 3: Step motor %d from %.2f to %.2f", motor + 1, (double)bg, (double)step_lvl);

	if (ramp_ms > 0) {
		const int ramp_steps = ramp_ms / 10;

		for (int i = 0; i <= ramp_steps && _state == TestState::RUNNING; i++) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during step ramp-up"); return; }

			float frac = (float)i / (float)ramp_steps;

			for (int m = 0; m < n; m++) {
				float val = (m == motor) ? (bg + (step_lvl - bg) * frac) : bg;
				commandMotor(m, val, hold_ms + ramp_ms + 500);
			}

			px4_usleep(10000);
		}
	}

	// Phase 4: Hold target at step level, all others at BT_BG_LVL
	const hrt_abstime hold_end = hrt_absolute_time() + (uint64_t)hold_ms * 1000ULL;

	while (hrt_absolute_time() < hold_end && _state == TestState::RUNNING) {
		if (isKillSwitchEngaged()) { abortTest("Kill switch during step hold"); return; }

		for (int m = 0; m < n; m++) {
			float val = (m == motor) ? step_lvl : bg;
			commandMotor(m, val, (uint32_t)((hold_end - hrt_absolute_time()) / 1000 + 500));
		}

		px4_usleep(50000);
	}

	// Phase 5: Ramp target motor back down to BT_BG_LVL
	if (ramp_ms > 0) {
		const int ramp_steps = ramp_ms / 10;

		for (int i = ramp_steps; i >= 0 && _state == TestState::RUNNING; i--) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during step ramp-down"); return; }

			float frac = (float)i / (float)ramp_steps;

			for (int m = 0; m < n; m++) {
				float val = (m == motor) ? (bg + (step_lvl - bg) * frac) : bg;
				commandMotor(m, val, ramp_ms + 500);
			}

			px4_usleep(10000);
		}
	}

	// Phase 6: Ramp all motors down to zero
	PX4_INFO("  Phase 6: Ramp all motors down to zero");

	if (ramp_ms > 0) {
		const int ramp_steps = ramp_ms / 10;

		for (int i = ramp_steps; i >= 0 && _state == TestState::RUNNING; i--) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during final ramp-down"); return; }

			float frac = (float)i / (float)ramp_steps;

			for (int m = 0; m < n; m++) {
				commandMotor(m, bg * frac, ramp_ms + 500);
			}

			px4_usleep(10000);
		}
	}

	releaseAllMotors();
	_state = TestState::IDLE;
	_active_test = TestType::NONE;
	PX4_INFO("Step+bg test complete");
}

/* ── Simultaneous step ────────────────────────────────────────────────── */

void BenchTest::runStepSimultaneous()
{
	if (!isDisarmed()) {
		PX4_ERR("Vehicle is armed – refusing to run bench test");
		return;
	}

	if (isKillSwitchEngaged()) {
		PX4_ERR("Kill switch engaged – refusing to start test");
		return;
	}

	const float level     = _param_bt_step_lvl.get();
	const float base      = _param_bt_base_lvl.get();
	const int   hold_ms   = _param_bt_step_dur.get();
	const int   ramp_ms   = _param_bt_ramp_time.get();
	const int   settle_ms = (base > 0.0f) ? _param_bt_bg_setl.get() : 0;
	const int   n         = _param_bt_num_motors.get();

	PX4_INFO("Simultaneous step: %d motors, base %.2f -> step %.2f, settle %d ms, hold %d ms, ramp %d ms",
		 n, (double)base, (double)level, settle_ms, hold_ms, ramp_ms);

	_state = TestState::RUNNING;
	_active_test = TestType::STEP_SIMULTANEOUS;
	_test_start_time = hrt_absolute_time();

	// Phase 1: Ramp all motors from zero to base level (if base > 0)
	if (base > 0.0f && ramp_ms > 0) {
		const int ramp_steps = ramp_ms / 10;

		for (int i = 0; i <= ramp_steps && _state == TestState::RUNNING; i++) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during simultaneous step pre-ramp"); return; }

			float frac = (float)i / (float)ramp_steps;

			for (int m = 0; m < n; m++) {
				commandMotor(m, base * frac, ramp_ms + settle_ms + hold_ms + 500);
			}

			px4_usleep(10000);
		}
	}

	// Phase 2: Hold all at base level for settle time
	if (settle_ms > 0) {
		PX4_INFO("  Settling at base %.2f for %d ms...", (double)base, settle_ms);
		const hrt_abstime settle_end = hrt_absolute_time() + (uint64_t)settle_ms * 1000ULL;

		while (hrt_absolute_time() < settle_end && _state == TestState::RUNNING) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during simultaneous step settle"); return; }

			for (int m = 0; m < n; m++) {
				commandMotor(m, base, (uint32_t)((settle_end - hrt_absolute_time()) / 1000 + 500));
			}

			px4_usleep(50000);
		}
	}

	if (_state != TestState::RUNNING) { return; }

	// Phase 3: Ramp all from base to step level
	if (ramp_ms > 0) {
		const int ramp_steps = ramp_ms / 10;

		for (int i = 0; i <= ramp_steps && _state == TestState::RUNNING; i++) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during simultaneous step ramp-up"); return; }

			float frac = (float)i / (float)ramp_steps;

			for (int m = 0; m < n; m++) {
				commandMotor(m, base + (level - base) * frac, ramp_ms + hold_ms + 500);
			}

			px4_usleep(10000);
		}
	}

	// Phase 4: Hold all at step level
	const hrt_abstime hold_end = hrt_absolute_time() + (uint64_t)hold_ms * 1000ULL;

	while (hrt_absolute_time() < hold_end && _state == TestState::RUNNING) {
		if (isKillSwitchEngaged()) { abortTest("Kill switch during simultaneous step hold"); return; }

		for (int m = 0; m < n; m++) {
			commandMotor(m, level, (uint32_t)((hold_end - hrt_absolute_time()) / 1000 + 500));
		}

		px4_usleep(50000);
	}

	// Phase 5: Ramp all back to zero
	if (ramp_ms > 0) {
		const int ramp_steps = ramp_ms / 10;

		for (int i = ramp_steps; i >= 0 && _state == TestState::RUNNING; i--) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during simultaneous step ramp-down"); return; }

			float frac = (float)i / (float)ramp_steps;

			for (int m = 0; m < n; m++) {
				commandMotor(m, base + (level - base) * frac, ramp_ms + 500);
			}

			px4_usleep(10000);
		}
	}

	releaseAllMotors();
	_state = TestState::IDLE;
	_active_test = TestType::NONE;
	PX4_INFO("Simultaneous step complete");
}

/* ── Simultaneous impulse ─────────────────────────────────────────────── */

void BenchTest::runImpulseSimultaneous()
{
	if (!isDisarmed()) {
		PX4_ERR("Vehicle is armed – refusing to run bench test");
		return;
	}

	if (isKillSwitchEngaged()) {
		PX4_ERR("Kill switch engaged – refusing to start test");
		return;
	}

	const float level     = _param_bt_imp_lvl.get();
	const float base      = _param_bt_base_lvl.get();
	const int   pulse_ms  = _param_bt_imp_dur.get();
	const int   ramp_ms   = _param_bt_ramp_time.get();
	const int   settle_ms = (base > 0.0f) ? _param_bt_bg_setl.get() : 0;
	const int   n         = _param_bt_num_motors.get();

	float impulse_lvl = base + level;
	if (impulse_lvl > 1.0f) { impulse_lvl = 1.0f; }

	PX4_INFO("Simultaneous impulse: %d motors, base %.2f, impulse %.2f, settle %d ms, pulse %d ms",
		 n, (double)base, (double)impulse_lvl, settle_ms, pulse_ms);

	_state = TestState::RUNNING;
	_active_test = TestType::IMPULSE_SIMULTANEOUS;
	_test_start_time = hrt_absolute_time();

	// Phase 1: Ramp all to base level first (if base > 0)
	if (base > 0.0f && ramp_ms > 0) {
		const int ramp_steps = ramp_ms / 10;

		for (int i = 0; i <= ramp_steps && _state == TestState::RUNNING; i++) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during simultaneous impulse pre-ramp"); return; }

			float frac = (float)i / (float)ramp_steps;

			for (int m = 0; m < n; m++) {
				commandMotor(m, base * frac, ramp_ms + settle_ms + pulse_ms + 500);
			}

			px4_usleep(10000);
		}
	}

	// Phase 2: Hold all at base level for settle time
	if (settle_ms > 0) {
		PX4_INFO("  Settling at base %.2f for %d ms...", (double)base, settle_ms);
		const hrt_abstime settle_end = hrt_absolute_time() + (uint64_t)settle_ms * 1000ULL;

		while (hrt_absolute_time() < settle_end && _state == TestState::RUNNING) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during simultaneous impulse settle"); return; }

			for (int m = 0; m < n; m++) {
				commandMotor(m, base, (uint32_t)((settle_end - hrt_absolute_time()) / 1000 + 500));
			}

			px4_usleep(50000);
		}
	}

	if (_state != TestState::RUNNING) { return; }

	// Phase 3: Fire all motors simultaneously at base + impulse level
	const hrt_abstime end = hrt_absolute_time() + (uint64_t)pulse_ms * 1000ULL;

	for (int m = 0; m < n; m++) {
		commandMotor(m, impulse_lvl, pulse_ms + 500);
	}

	while (hrt_absolute_time() < end && _state == TestState::RUNNING) {
		if (isKillSwitchEngaged()) { abortTest("Kill switch during simultaneous impulse"); return; }

		px4_usleep(10000);
	}

	// Phase 4: Ramp all back to zero
	if (base > 0.0f && ramp_ms > 0) {
		const int ramp_steps = ramp_ms / 10;

		for (int i = ramp_steps; i >= 0 && _state == TestState::RUNNING; i--) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during simultaneous impulse ramp-down"); return; }

			float frac = (float)i / (float)ramp_steps;

			for (int m = 0; m < n; m++) {
				commandMotor(m, base * frac, ramp_ms + 500);
			}

			px4_usleep(10000);
		}
	}

	releaseAllMotors();
	_state = TestState::IDLE;
	_active_test = TestType::NONE;
	PX4_INFO("Simultaneous impulse complete");
}

/* ── Flight test ──────────────────────────────────────────────────────── */

void BenchTest::runFlightTest()
{
	if (!isDisarmed()) {
		PX4_ERR("Vehicle is armed – refusing to run flight test");
		return;
	}

	if (isKillSwitchEngaged()) {
		PX4_ERR("Kill switch engaged – refusing to start flight test");
		return;
	}

	const float hover_lvl  = _param_bt_flt_hovr.get();
	const int   hover_ms   = _param_bt_flt_hovt.get();
	const int   blip_count = _param_bt_flt_blpn.get();
	const float blip_amp   = _param_bt_flt_blpa.get();
	const int   blip_dur   = _param_bt_flt_blpd.get();
	const int   blip_int   = _param_bt_flt_blpi.get();
	const int   ramp_ms    = _param_bt_ramp_time.get();
	const int   n          = _param_bt_num_motors.get();

	PX4_INFO("Flight test: hover %.2f, settle %d ms, %d blips (amp %.2f, dur %d ms, interval %d ms)",
		 (double)hover_lvl, hover_ms, blip_count, (double)blip_amp, blip_dur, blip_int);

	_state = TestState::RUNNING;
	_active_test = TestType::FLIGHT;
	_test_start_time = hrt_absolute_time();

	/* ── Phase 1: Ramp up all motors to hover throttle ──────────── */
	PX4_INFO("  Phase 1: Ramp to hover throttle (%.2f)", (double)hover_lvl);

	if (ramp_ms > 0) {
		const int ramp_steps = ramp_ms / 10;

		for (int i = 0; i <= ramp_steps && _state == TestState::RUNNING; i++) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during flight ramp-up"); return; }

			float frac = (float)i / (float)ramp_steps;

			for (int m = 0; m < n; m++) {
				commandMotor(m, hover_lvl * frac, ramp_ms + hover_ms + 2000);
			}

			px4_usleep(10000);
		}
	}

	/* ── Phase 2: Hold hover for settle time ────────────────────── */
	PX4_INFO("  Phase 2: Hover settle for %d ms", hover_ms);
	{
		const hrt_abstime hover_end = hrt_absolute_time() + (uint64_t)hover_ms * 1000ULL;

		while (hrt_absolute_time() < hover_end && _state == TestState::RUNNING) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during hover settle"); return; }

			for (int m = 0; m < n; m++) {
				commandMotor(m, hover_lvl, (uint32_t)((hover_end - hrt_absolute_time()) / 1000 + 2000));
			}

			px4_usleep(50000);
		}
	}

	if (_state != TestState::RUNNING) { return; }

	/* ── Phase 3: Orthogonal multisine excitation ───────────────── */
	PX4_INFO("  Phase 3: Multisine excitation (reading DTRG_MSINE_* params)");
	{
		/* Read multisine parameters from the multisine_excitation module */
		int32_t msine_nmot = n;
		float   msine_amp  = 0.05f;
		float   msine_t    = 15.0f;
		float   msine_fmin = 0.1f;
		float   msine_fmax = 1.0f;
		int32_t msine_seq  = 1;

		param_t h;
		h = param_find("DTRG_MSINE_NMOT"); if (h != PARAM_INVALID) { param_get(h, &msine_nmot); }
		h = param_find("DTRG_MSINE_AMP");  if (h != PARAM_INVALID) { param_get(h, &msine_amp); }
		h = param_find("DTRG_MSINE_T");    if (h != PARAM_INVALID) { param_get(h, &msine_t); }
		h = param_find("DTRG_MSINE_FMIN"); if (h != PARAM_INVALID) { param_get(h, &msine_fmin); }
		h = param_find("DTRG_MSINE_FMAX"); if (h != PARAM_INVALID) { param_get(h, &msine_fmax); }
		h = param_find("DTRG_MSINE_SEQ");  if (h != PARAM_INVALID) { param_get(h, &msine_seq); }

		/* Limit motor count to our configured number */
		if (msine_nmot > n) { msine_nmot = n; }

		/* Configure and run the multisine generator on the heap */
		multisine::MultisineExcitation *gen = new multisine::MultisineExcitation();

		if (!gen) {
			PX4_ERR("  Failed to allocate multisine generator");

		} else {
			float period_per_motor = msine_seq ? (msine_t / (float)msine_nmot) : msine_t;

			if (!gen->configure((uint8_t)msine_nmot, period_per_motor, msine_fmin, msine_fmax, msine_amp)) {
				PX4_ERR("  Failed to configure multisine generator");

			} else {
				gen->setSequentialMode(msine_seq != 0);
				float total_dur = gen->getTotalDuration();

				PX4_INFO("  Multisine: %d motors, %.1fs total, %.2f-%.2f Hz, amp %.3f, %s",
					 (int)msine_nmot, (double)total_dur,
					 (double)msine_fmin, (double)msine_fmax, (double)msine_amp,
					 msine_seq ? "sequential" : "simultaneous");

				gen->start();

				const uint32_t update_interval_us = 4000; /* 250 Hz like the multisine module */
				const float dt_s = (float)update_interval_us / 1e6f;
				hrt_abstime last_print = hrt_absolute_time();

				while (gen->isActive() && _state == TestState::RUNNING) {
					if (isKillSwitchEngaged()) {
						gen->stop();
						delete gen;
						abortTest("Kill switch during multisine excitation");
						return;
					}

					float excitation[multisine::MAX_MOTORS] = {};
					gen->update(dt_s, excitation);

					/* Apply hover + excitation to each motor */
					for (int m = 0; m < n; m++) {
						float val = hover_lvl;

						if (m < msine_nmot) {
							val += excitation[m];
						}

						if (val < 0.0f) { val = 0.0f; }
						if (val > 1.0f) { val = 1.0f; }

						commandMotor(m, val, 200);
					}

					/* Print progress every 2 seconds */
					hrt_abstime now = hrt_absolute_time();

					if ((now - last_print) > 2000000) {
						float elapsed = gen->getElapsedTime();
						PX4_INFO("    Multisine: %.1f/%.1fs  motor %d",
							 (double)elapsed, (double)total_dur,
							 (int)gen->getCurrentMotor());
						last_print = now;
					}

					px4_usleep(update_interval_us);
				}

				PX4_INFO("  Multisine excitation complete");
			}

			delete gen;
		}
	}

	if (_state != TestState::RUNNING) { return; }

	/* Brief settle at hover before blips */
	{
		const hrt_abstime settle_end = hrt_absolute_time() + 500000ULL; /* 0.5 s */

		while (hrt_absolute_time() < settle_end && _state == TestState::RUNNING) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during post-multisine settle"); return; }

			for (int m = 0; m < n; m++) {
				commandMotor(m, hover_lvl, 1000);
			}

			px4_usleep(50000);
		}
	}

	if (_state != TestState::RUNNING) { return; }

	/* ── Phase 4: Periodic thrust blips ─────────────────────────── */
	PX4_INFO("  Phase 4: %d thrust blips (amp %.2f, dur %d ms, interval %d ms)",
		 blip_count, (double)blip_amp, blip_dur, blip_int);

	for (int blip = 0; blip < blip_count && _state == TestState::RUNNING; blip++) {

		PX4_INFO("    Blip %d/%d", blip + 1, blip_count);

		/* Blip: apply hover + blip_amp to all motors */
		float blip_level = hover_lvl + blip_amp;

		if (blip_level > 1.0f) { blip_level = 1.0f; }

		{
			const hrt_abstime blip_end = hrt_absolute_time() + (uint64_t)blip_dur * 1000ULL;

			while (hrt_absolute_time() < blip_end && _state == TestState::RUNNING) {
				if (isKillSwitchEngaged()) { abortTest("Kill switch during blip"); return; }

				for (int m = 0; m < n; m++) {
					commandMotor(m, blip_level, blip_dur + 500);
				}

				px4_usleep(20000); /* 50 Hz refresh */
			}
		}

		/* Return to hover for the inter-blip interval */
		if (blip < blip_count - 1 && _state == TestState::RUNNING) {
			const hrt_abstime interval_end = hrt_absolute_time() + (uint64_t)blip_int * 1000ULL;

			while (hrt_absolute_time() < interval_end && _state == TestState::RUNNING) {
				if (isKillSwitchEngaged()) { abortTest("Kill switch during blip interval"); return; }

				for (int m = 0; m < n; m++) {
					commandMotor(m, hover_lvl, (uint32_t)((interval_end - hrt_absolute_time()) / 1000 + 500));
				}

				px4_usleep(50000);
			}
		}
	}

	/* ── Phase 5: Ramp down ─────────────────────────────────────── */
	PX4_INFO("  Phase 5: Ramp down");

	if (ramp_ms > 0 && _state == TestState::RUNNING) {
		const int ramp_steps = ramp_ms / 10;

		for (int i = ramp_steps; i >= 0 && _state == TestState::RUNNING; i--) {
			if (isKillSwitchEngaged()) { abortTest("Kill switch during flight ramp-down"); return; }

			float frac = (float)i / (float)ramp_steps;

			for (int m = 0; m < n; m++) {
				commandMotor(m, hover_lvl * frac, ramp_ms + 500);
			}

			px4_usleep(10000);
		}
	}

	releaseAllMotors();
	_state = TestState::IDLE;
	_active_test = TestType::NONE;
	PX4_INFO("Flight test complete");
}

void BenchTest::runAllMotorsSequential(void (BenchTest::*singleTestFn)(int))
{
	const int n = _param_bt_num_motors.get();
	const int delay_ms = _param_bt_inter_dly.get();

	for (int m = 0; m < n && _state != TestState::ABORTING; m++) {
		PX4_INFO("--- Motor %d/%d ---", m + 1, n);
		(this->*singleTestFn)(m);

		if (m < n - 1 && _state != TestState::ABORTING) {
			PX4_INFO("  Inter-motor delay %d ms", delay_ms);
			px4_usleep((uint64_t)delay_ms * 1000ULL);
		}
	}
}

/* ── ModuleBase interface ─────────────────────────────────────────────── */

int BenchTest::task_spawn(int argc, char *argv[])
{
	BenchTest *instance = new BenchTest();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int BenchTest::print_status()
{
	const char *state_str = "UNKNOWN";

	switch (_state) {
	case TestState::IDLE:     state_str = "IDLE";     break;

	case TestState::RUNNING:  state_str = "RUNNING";  break;

	case TestState::ABORTING: state_str = "ABORTING"; break;
	}

	PX4_INFO("State: %s", state_str);
	PX4_INFO("Motors configured: %" PRId32, _param_bt_num_motors.get());
	PX4_INFO("Kill switch: %s", isKillSwitchEngaged() ? "ENGAGED" : "normal");
	PX4_INFO("Disarmed: %s", isDisarmed() ? "yes" : "NO (armed)");

	// Print latest ESC telemetry if available
	esc_status_s esc;

	if (_esc_status_sub.copy(&esc)) {
		PX4_INFO("ESC count: %d, online flags: 0x%02x", esc.esc_count, esc.esc_online_flags);

		for (int i = 0; i < esc.esc_count && i < 8; i++) {
			PX4_INFO("  ESC%d: %" PRId32 " RPM, %.2fV, %.2fA",
				 i, esc.esc[i].esc_rpm,
				 (double)esc.esc[i].esc_voltage,
				 (double)esc.esc[i].esc_current);
		}
	}

	// Print power monitor if available
	power_monitor_s pmon;

	if (_power_monitor_sub.copy(&pmon)) {
		PX4_INFO("Power: %.2fV  %.2fA  %.2fW",
			 (double)pmon.voltage_v, (double)pmon.current_a, (double)pmon.power_w);
	}

	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int BenchTest::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running – start the module first with: bench_test start");
		return 1;
	}

	BenchTest *obj = get_instance();

	if (!obj) {
		PX4_ERR("module instance not available");
		return 1;
	}

	/* ── Parse optional -m <motor> argument ─────────────────────── */
	int motor = -1; // -1 means not specified
	int myoptind = 0;
	int ch;
	const char *myoptarg = nullptr;

	// Find the subcommand first (it's argv[0])
	const char *subcmd = argv[0];

	// Shift past the subcommand for option parsing
	int opt_argc = argc - 1;
	char **opt_argv = argv + 1;
	myoptind = 0;

	while ((ch = px4_getopt(opt_argc, opt_argv, "m:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'm':
			motor = (int)strtol(myoptarg, nullptr, 10) - 1; // user provides 1-based

			if (motor < 0 || motor >= obj->_param_bt_num_motors.get()) {
				PX4_ERR("Invalid motor %d (valid: 1-%" PRId32 ")", motor + 1, obj->_param_bt_num_motors.get());
				return 1;
			}

			break;

		default:
			return print_usage("unknown option");
		}
	}

	/* ── Dispatch subcommands ───────────────────────────────────── */

	if (!strcmp(subcmd, "step")) {
		if (motor < 0) {
			PX4_ERR("Specify motor with -m <1..N>");
			return 1;
		}

		obj->runStepTest(motor);
		return 0;
	}

	if (!strcmp(subcmd, "step_all")) {
		obj->_state = TestState::RUNNING;
		obj->_active_test = TestType::STEP_ALL;
		obj->runAllMotorsSequential(&BenchTest::runStepTest);
		return 0;
	}

	if (!strcmp(subcmd, "step_idle")) {
		if (motor < 0) {
			PX4_ERR("Specify motor with -m <1..N>");
			return 1;
		}

		obj->runStepWithIdleTest(motor);
		return 0;
	}

	if (!strcmp(subcmd, "step_sim")) {
		obj->runStepSimultaneous();
		return 0;
	}

	if (!strcmp(subcmd, "impulse")) {
		if (motor < 0) {
			PX4_ERR("Specify motor with -m <1..N>");
			return 1;
		}

		obj->runImpulseTest(motor);
		return 0;
	}

	if (!strcmp(subcmd, "impulse_all")) {
		obj->_state = TestState::RUNNING;
		obj->_active_test = TestType::IMPULSE_ALL;
		obj->runAllMotorsSequential(&BenchTest::runImpulseTest);
		return 0;
	}

	if (!strcmp(subcmd, "impulse_sim")) {
		obj->runImpulseSimultaneous();
		return 0;
	}

	if (!strcmp(subcmd, "tweet")) {
		if (motor < 0) {
			PX4_ERR("Specify motor with -m <1..N>");
			return 1;
		}

		obj->runTweetTest(motor);
		return 0;
	}

	if (!strcmp(subcmd, "tweet_all")) {
		obj->_state = TestState::RUNNING;
		obj->_active_test = TestType::TWEET_ALL;
		obj->runAllMotorsSequential(&BenchTest::runTweetTest);
		return 0;
	}

	if (!strcmp(subcmd, "multi")) {
		obj->runMultiTest();
		return 0;
	}

	if (!strcmp(subcmd, "sweep")) {
		if (motor < 0) {
			PX4_ERR("Specify motor with -m <1..N>");
			return 1;
		}

		obj->runCurrentSweep(motor);
		return 0;
	}

	if (!strcmp(subcmd, "sweep_all")) {
		obj->_state = TestState::RUNNING;
		obj->_active_test = TestType::CURRENT_SWEEP_ALL;
		obj->runAllMotorsSequential(&BenchTest::runCurrentSweep);
		return 0;
	}

	if (!strcmp(subcmd, "flight")) {
		obj->runFlightTest();
		return 0;
	}

	if (!strcmp(subcmd, "abort")) {
		obj->abortTest("User abort");
		return 0;
	}

	return print_usage("unknown command");
}

int BenchTest::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Motor bench testing module for pre-arm motor excitation tests.

Runs step, impulse, chirp/tweet, multi-motor, current-sweep, and flight
tests while the vehicle remains DISARMED. Uses actuator_test messages so the
ESC driver handles the actual output. The normal logger captures
high-rate eRPM (bidirectional DShot) and power-module telemetry.

The radio kill-switch (manual_control_switches) is polled every cycle
and will immediately abort any running test – no MAVLink dependency.

WARNING: Remove all propellers before running bench tests!

### Examples

Start the module:
$ bench_test start

Run a step test on motor 2:
$ bench_test step -m 2

Step motor 2 while all others spin at idle:
$ bench_test step_idle -m 2

Step all 8 motors at once:
$ bench_test step_sim

Run impulse test on all motors sequentially:
$ bench_test impulse_all

Impulse all motors at once:
$ bench_test impulse_sim

Run chirp/tweet sweep on motor 1:
$ bench_test tweet -m 1

Run all motors simultaneously:
$ bench_test multi

Run current-draw sweep on motor 3:
$ bench_test sweep -m 3

Run full flight test (takeoff, multisine excitation, thrust blips):
$ bench_test flight

Abort a running test:
$ bench_test abort

Check status and ESC telemetry:
$ bench_test status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("bench_test", "module");
	PRINT_MODULE_USAGE_COMMAND("start");

	PRINT_MODULE_USAGE_COMMAND_DESCR("step", "Single motor step test (ramp-up, hold, ramp-down)");
	PRINT_MODULE_USAGE_PARAM_INT('m', 1, 1, 8, "Motor number (1-based)", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("step_all", "Step test on all motors sequentially");

	PRINT_MODULE_USAGE_COMMAND_DESCR("step_idle", "Step one motor while holding all others at BT_IDLE_LVL");
	PRINT_MODULE_USAGE_PARAM_INT('m', 1, 1, 8, "Motor number (1-based)", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("step_sim", "Step all motors simultaneously to BT_STEP_LVL");

	PRINT_MODULE_USAGE_COMMAND_DESCR("impulse", "Single motor short impulse burst");
	PRINT_MODULE_USAGE_PARAM_INT('m', 1, 1, 8, "Motor number (1-based)", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("impulse_all", "Impulse test on all motors sequentially");

	PRINT_MODULE_USAGE_COMMAND_DESCR("impulse_sim", "Impulse all motors simultaneously");

	PRINT_MODULE_USAGE_COMMAND_DESCR("tweet", "Single motor chirp/tweet frequency sweep");
	PRINT_MODULE_USAGE_PARAM_INT('m', 1, 1, 8, "Motor number (1-based)", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("tweet_all", "Chirp/tweet test on all motors sequentially");

	PRINT_MODULE_USAGE_COMMAND_DESCR("multi", "All motors simultaneously at set level");

	PRINT_MODULE_USAGE_COMMAND_DESCR("sweep", "Single motor current-draw staircase sweep");
	PRINT_MODULE_USAGE_PARAM_INT('m', 1, 1, 8, "Motor number (1-based)", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("sweep_all", "Current sweep on all motors sequentially");

	PRINT_MODULE_USAGE_COMMAND_DESCR("flight", "Full flight test: takeoff, multisine excitation, thrust blips");

	PRINT_MODULE_USAGE_COMMAND_DESCR("abort", "Immediately stop any running test");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/* ── Module entry point ───────────────────────────────────────────────── */

extern "C" __EXPORT int bench_test_main(int argc, char *argv[])
{
	return BenchTest::main(argc, argv);
}

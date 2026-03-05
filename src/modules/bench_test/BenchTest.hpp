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
 * @file BenchTest.hpp
 *
 * Motor bench testing module. Runs pre-arm motor excitation tests
 * (step, impulse, chirp/tweet, current-sweep) while maintaining
 * high-rate logging and radio kill-switch monitoring.
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <math.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_test.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/bench_test_vc_status.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/manual_control_switches.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/power_monitor.h>
#include <uORB/topics/multisine_excitation_status.h>

using namespace time_literals;

/* ── Voltage compensator ────────────────────────────────────────────────
 *
 * Adjusts the motor command δ to counteract battery voltage sag so
 * that the effective voltage δ·Vb stays constant regardless of SoC
 * and current draw.
 *
 * Algorithm (forward Euler, called at the motor-command rate):
 *   1. V_delta = δ · Vb_op
 *   2. ω_k    = (θw1·Vδ + θw2·√Vδ + θw3 − θw4·ω_{k-1}) / (1+θw4)
 *   3. I_rot  = θI1·ω + θI2·ω² + θI3     (per rotor)
 *   4. I_tot  = Σ I_rot
 *   5. V0, R0, R1, τ1 from 3rd-order polynomials in SoC
 *   6. V_RC_k = V_RC_{k-1} + Ts·(R1/τ1·I − 1/τ1·V_RC_{k-1})
 *   7. Vb_k   = V0 − I·R0 − V_RC_k
 *   8. δ'     = (Vb_op / Vb_k) · δ
 * ──────────────────────────────────────────────────────────────────── */

#include <VoltageCompensator.hpp>

extern "C" __EXPORT int bench_test_main(int argc, char *argv[]);

class BenchTest : public ModuleBase<BenchTest>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	BenchTest();
	~BenchTest() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	/** Scheduled work item callback – checks kill switch & runs active test state machine. */
	void Run() override;

	/** Release all motors (set NaN = disarmed). */
	void releaseAllMotors();

	/** Publish actuator_test for a single motor. */
	void commandMotor(int motor_index, float value, uint32_t timeout_ms);

	/** Check the kill switch via manual_control_switches (RC-based, no MAVLink dependency). */
	bool isKillSwitchEngaged();

	/** Check whether the vehicle is disarmed (safe to run bench tests). */
	bool isDisarmed();

	/** Abort the running test and release all motors. */
	void abortTest(const char *reason);

	/* ── Test execution helpers ─────────────────────────────────── */

	/** Single-motor step test: ramp up, hold, ramp down. */
	void runStepTest(int motor);

	/** Single-motor impulse (short burst). */
	void runImpulseTest(int motor);

	/** Single-motor chirp / tweet frequency sweep. */
	void runTweetTest(int motor);

	/** All motors simultaneously at a set level. */
	void runMultiTest();

	/** Single-motor step test with all other motors held at idle. */
	void runStepWithIdleTest(int motor);

	/** All motors step to the same level simultaneously. */
	void runStepSimultaneous();

	/** All motors impulse simultaneously. */
	void runImpulseSimultaneous();

	/** Single-motor current-draw sweep (staircase throttle). */
	void runCurrentSweep(int motor);

	/** Full flight test: takeoff, hover, multisine excitation, thrust blips, land. */
	void runFlightTest();

	/** Sequentially run a single-motor test on every motor. */
	void runAllMotorsSequential(void (BenchTest::*singleTestFn)(int));

	/* ── State machine ──────────────────────────────────────────── */

	enum class TestState : uint8_t {
		IDLE = 0,
		RUNNING,
		ABORTING,
	};

	enum class TestType : uint8_t {
		NONE = 0,
		STEP_SINGLE,
		STEP_ALL,
		STEP_IDLE_BG,        /**< step one motor, all others at idle */
		STEP_SIMULTANEOUS,   /**< all motors step together */
		IMPULSE_SINGLE,
		IMPULSE_ALL,
		IMPULSE_SIMULTANEOUS,/**< all motors impulse together */
		TWEET_SINGLE,
		TWEET_ALL,
		MULTI,
		CURRENT_SWEEP_SINGLE,
		CURRENT_SWEEP_ALL,
		FLIGHT,
	};

	TestState _state{TestState::IDLE};
	TestType  _active_test{TestType::NONE};
	int       _target_motor{0};            /**< 0-based motor index for single tests */
	hrt_abstime _test_start_time{0};       /**< start timestamp of current test / phase */
	int       _test_phase{0};              /**< sub-phase counter (e.g. sweep step index, motor index in seq) */

	/* ── Subscriptions ──────────────────────────────────────────── */
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription         _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription         _esc_status_sub{ORB_ID(esc_status)};
	uORB::Subscription         _manual_switches_sub{ORB_ID(manual_control_switches)};
	uORB::Subscription         _power_monitor_sub{ORB_ID(power_monitor)};
	uORB::Subscription         _multisine_status_sub{ORB_ID(multisine_excitation_status)};

	/* ── Publication ────────────────────────────────────────────── */
	uORB::Publication<actuator_test_s>       _actuator_test_pub{ORB_ID(actuator_test)};
	uORB::Publication<bench_test_vc_status_s> _vc_status_pub{ORB_ID(bench_test_vc_status)};

	/* ── Performance counters ───────────────────────────────────── */
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	/* ── Parameters ─────────────────────────────────────────────── */
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::BT_NUM_MOTORS>)   _param_bt_num_motors,
		(ParamFloat<px4::params::BT_BG_LVL>)     _param_bt_bg_lvl,
		(ParamInt<px4::params::BT_BG_SETL>)      _param_bt_bg_setl,
		(ParamFloat<px4::params::BT_BASE_LVL>)   _param_bt_base_lvl,
		(ParamFloat<px4::params::BT_STEP_LVL>)   _param_bt_step_lvl,
		(ParamInt<px4::params::BT_STEP_DUR>)    _param_bt_step_dur,
		(ParamFloat<px4::params::BT_IMP_LVL>)   _param_bt_imp_lvl,
		(ParamInt<px4::params::BT_IMP_DUR>)     _param_bt_imp_dur,
		(ParamFloat<px4::params::BT_TWE_F_STA>) _param_bt_twe_f_sta,
		(ParamFloat<px4::params::BT_TWE_F_END>) _param_bt_twe_f_end,
		(ParamFloat<px4::params::BT_TWE_AMP>)   _param_bt_twe_amp,
		(ParamFloat<px4::params::BT_TWE_BIAS>)  _param_bt_twe_bias,
		(ParamInt<px4::params::BT_TWE_DUR>)     _param_bt_twe_dur,
		(ParamFloat<px4::params::BT_MULTI_LVL>) _param_bt_multi_lvl,
		(ParamInt<px4::params::BT_MULTI_DUR>)   _param_bt_multi_dur,
		(ParamFloat<px4::params::BT_ISWEEP_STA>) _param_bt_isweep_sta,
		(ParamFloat<px4::params::BT_ISWEEP_END>) _param_bt_isweep_end,
		(ParamInt<px4::params::BT_ISWEEP_N>)    _param_bt_isweep_n,
		(ParamInt<px4::params::BT_ISWEEP_DWL>)  _param_bt_isweep_dwl,
		(ParamInt<px4::params::BT_RAMP_TIME>)   _param_bt_ramp_time,
		(ParamInt<px4::params::BT_INTER_DLY>)   _param_bt_inter_dly,
		(ParamFloat<px4::params::BT_FLT_HOVR>)  _param_bt_flt_hovr,
		(ParamInt<px4::params::BT_FLT_HOVT>)    _param_bt_flt_hovt,
		(ParamInt<px4::params::BT_FLT_BLPN>)    _param_bt_flt_blpn,
		(ParamFloat<px4::params::BT_FLT_BLPA>)  _param_bt_flt_blpa,
		(ParamInt<px4::params::BT_FLT_BLPD>)    _param_bt_flt_blpd,
		(ParamInt<px4::params::BT_FLT_BLPI>)    _param_bt_flt_blpi,
		(ParamFloat<px4::params::BT_FLT_BLPS>)  _param_bt_flt_blps,
		(ParamFloat<px4::params::BT_FLT_HIHI>)  _param_bt_flt_hihi,
		(ParamFloat<px4::params::BT_FLT_LOLO>)  _param_bt_flt_lolo,
		(ParamInt<px4::params::BT_FLT_STPH>)    _param_bt_flt_stph,
		(ParamInt<px4::params::BT_FLT_STPR>)    _param_bt_flt_stpr,
		(ParamFloat<px4::params::BT_FLT_VMIN>)  _param_bt_flt_vmin,

		/* ── Voltage compensator parameters ─────────────────────── */
		(ParamFloat<px4::params::BT_VC_VBOP>)   _param_bt_vc_vbop,
		(ParamFloat<px4::params::BT_VC_TW1>)    _param_bt_vc_tw1,
		(ParamFloat<px4::params::BT_VC_TW2>)    _param_bt_vc_tw2,
		(ParamFloat<px4::params::BT_VC_TW3>)    _param_bt_vc_tw3,
		(ParamFloat<px4::params::BT_VC_TW4>)    _param_bt_vc_tw4,
		(ParamFloat<px4::params::BT_VC_TI1>)    _param_bt_vc_ti1,
		(ParamFloat<px4::params::BT_VC_TI2>)    _param_bt_vc_ti2,
		(ParamFloat<px4::params::BT_VC_TI3>)    _param_bt_vc_ti3,
		(ParamFloat<px4::params::BT_VC_V0C0>)   _param_bt_vc_v0c0,
		(ParamFloat<px4::params::BT_VC_V0C1>)   _param_bt_vc_v0c1,
		(ParamFloat<px4::params::BT_VC_V0C2>)   _param_bt_vc_v0c2,
		(ParamFloat<px4::params::BT_VC_V0C3>)   _param_bt_vc_v0c3,
		(ParamFloat<px4::params::BT_VC_R0C0>)   _param_bt_vc_r0c0,
		(ParamFloat<px4::params::BT_VC_R0C1>)   _param_bt_vc_r0c1,
		(ParamFloat<px4::params::BT_VC_R0C2>)   _param_bt_vc_r0c2,
		(ParamFloat<px4::params::BT_VC_R0C3>)   _param_bt_vc_r0c3,
		(ParamFloat<px4::params::BT_VC_R1C0>)   _param_bt_vc_r1c0,
		(ParamFloat<px4::params::BT_VC_R1C1>)   _param_bt_vc_r1c1,
		(ParamFloat<px4::params::BT_VC_R1C2>)   _param_bt_vc_r1c2,
		(ParamFloat<px4::params::BT_VC_R1C3>)   _param_bt_vc_r1c3,
		(ParamFloat<px4::params::BT_VC_T1C0>)   _param_bt_vc_t1c0,
		(ParamFloat<px4::params::BT_VC_T1C1>)   _param_bt_vc_t1c1,
		(ParamFloat<px4::params::BT_VC_T1C2>)   _param_bt_vc_t1c2,
		(ParamFloat<px4::params::BT_VC_T1C3>)   _param_bt_vc_t1c3,
		(ParamFloat<px4::params::BT_VC_MAXCMD>) _param_bt_vc_maxcmd
	)

	/* ── Voltage compensator runtime state ──────────────────────── */
	bool               _compensator_enabled{false};   /**< set by -c CLI flag */
	bool               _compensator_simple{false};    /**< set by -s CLI flag: use measured voltage instead of predicted */
	VoltageCompensator _voltage_compensator{};         /**< compensator instance */
	hrt_abstime        _vc_last_update{0};             /**< timestamp of last compensator update */

	/** Battery subscriptions for compensator SoC reading */
	uORB::Subscription _vc_batt_subs[4] {
		{ORB_ID(battery_status), 0},
		{ORB_ID(battery_status), 1},
		{ORB_ID(battery_status), 2},
		{ORB_ID(battery_status), 3},
	};

	/**
	 * Load compensator coefficients from parameters.
	 * Call once before a compensator-enabled test.
	 */
	void loadCompensatorParams();

	/**
	 * Read current battery SoC from the first valid battery_status subscription.
	 * @return SoC in [0, 1], or -1.0f if unavailable.
	 */
	float readBatterySoC();

	/**
	 * Read the measured terminal voltage from the first valid battery_status subscription.
	 * @return voltage in Volts, or -1.0f if unavailable.
	 */
	float readBatteryVoltage();

	/**
	 * Command a motor, routing it through the voltage compensator when active.
	 *
	 * @param motor_index  0-based motor to command
	 * @param value        raw normalised command [0, 1]
	 * @param timeout_ms   actuator_test timeout
	 * @param is_target    true  → apply compensation & publish log message
	 *                     false → store delta for battery model, pass through raw
	 */
	void compensatedCommandMotor(int motor_index, float value, uint32_t timeout_ms,
				     bool is_target = true, float delta_bg = 0.0f, int n_bg = 0);
};

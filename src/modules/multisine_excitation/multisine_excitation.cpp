/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file multisine_excitation.cpp
 *
 * Multisine Excitation Module implementation
 */

#include "multisine_excitation.hpp"

#include <mathlib/mathlib.h>
#include <cmath>
#include <fcntl.h>
#include <errno.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_test.h>
#include <px4_platform_common/param.h>

MultisineExcitationModule::MultisineExcitationModule() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	_status_pub.advertise();
}

MultisineExcitationModule::~MultisineExcitationModule()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool MultisineExcitationModule::init()
{
	// Initialize with current parameters
	parameters_updated();

	// Configure the generator
	if (!configureGenerator()) {
		PX4_WARN("Failed to configure generator with default parameters");
		// Don't fail init, parameters might be changed later
	}

	// Schedule at 250Hz (4ms interval)
	ScheduleOnInterval(4000_us);

	return true;
}

void MultisineExcitationModule::parameters_updated()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
	}
}

bool MultisineExcitationModule::configureGenerator()
{
	// Calculate period per motor based on mode
	float total_time = _param_period.get();
	float period_per_motor;

	if (_param_sequential.get()) {
		// Sequential mode: divide total time among motors
		period_per_motor = total_time / static_cast<float>(_param_num_motors.get());
	} else {
		// Simultaneous mode: use full total time for each motor
		period_per_motor = total_time;
	}

	bool success = _generator.configure(
		       static_cast<uint8_t>(_param_num_motors.get()),
		       period_per_motor,
		       _param_freq_min.get(),
		       _param_freq_max.get(),
		       _param_amplitude.get()
	       );

	if (success) {
		// Set sequential mode after successful configuration
		_generator.setSequentialMode(_param_sequential.get());
	}

	return success;
}

float MultisineExcitationModule::getRcChannelValue() const
{
	int channel = _param_rc_channel.get();

	if (channel < 1 || channel > 18) {
		return 0.0f;
	}

	if (_input_rc.channel_count < channel) {
		return 0.0f;
	}

	// Convert from PWM (1000-2000us) to normalized (0-1)
	uint16_t pwm_value = _input_rc.values[channel - 1];  // channels are 0-indexed in array

	// Clamp to typical RC range
	if (pwm_value < 1000) pwm_value = 1000;
	if (pwm_value > 2000) pwm_value = 2000;

	return (pwm_value - 1000.0f) / 1000.0f;  // Convert to 0-1 range
}

bool MultisineExcitationModule::isRcTriggered() const
{
	return getRcChannelValue() > 0.5f;
}

void MultisineExcitationModule::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain((now - _last_run) * 1e-6f, 0.001f, 0.1f);
	_last_run = now;

	// Check for parameter updates
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
		parameters_updated();

		// Reconfigure generator if not actively exciting
		if (_phase == Phase::IDLE) {
			configureGenerator();
		}
	}

	// Module disabled - publish inactive status and return
	if (!_param_enable.get()) {
		float excitation[multisine::MAX_MOTORS] = {};
		publishStatus(excitation);
		perf_end(_loop_perf);
		return;
	}

	// Update vehicle status (armed state)
	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			bool was_armed = _armed;
			_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

			// Safety: stop excitation if disarmed
			if (was_armed && !_armed && _phase != Phase::IDLE) {
				PX4_INFO("Disarmed - stopping excitation");
				_generator.stop();
				_phase = Phase::IDLE;
				_manual_trigger = false;
			}
		}
	}

	// Update manual control setpoint (for RC aux channel)
	_input_rc_sub.copy(&_input_rc);

	// Check for RC trigger
	bool rc_triggered = isRcTriggered();
	bool trigger_rising_edge = rc_triggered && !_rc_triggered_prev;
	bool trigger_falling_edge = !rc_triggered && _rc_triggered_prev;
	_rc_triggered_prev = rc_triggered;

	// State machine for excitation phases
	float excitation[multisine::MAX_MOTORS] = {};

	switch (_phase) {
	case Phase::IDLE:
		// Check for trigger (RC switch or manual command)
		if (_armed && (trigger_rising_edge || _manual_trigger)) {
			// Reconfigure generator before starting
			if (!configureGenerator()) {
				PX4_ERR("Failed to configure generator");
				_manual_trigger = false;
				break;
			}

			PX4_INFO("Starting excitation: %d motors, %.1fs period, %.2f-%.2f Hz",
				 (int)_param_num_motors.get(), (double)_param_period.get(),
				 (double)_param_freq_min.get(), (double)_param_freq_max.get());

			_phase = Phase::PRE_SETTLE;
			_settle_time = 0.0f;
		}

		break;

	case Phase::PRE_SETTLE:
		// Wait for settle time before starting excitation
		_settle_time += dt;

		if (_settle_time >= SETTLE_DURATION_S) {
			_generator.start();
			_phase = Phase::EXCITING;
			PX4_INFO("Pre-settle complete, starting signal generation");
		}

		// Check for abort (RC released or disarmed)
		if (!_armed || trigger_falling_edge) {
			PX4_INFO("Excitation aborted during pre-settle");
			_phase = Phase::IDLE;
			_manual_trigger = false;
		}

		break;

	case Phase::EXCITING:
		// Update generator and get excitation values
		if (!_generator.update(dt, excitation)) {
			// Excitation complete
			PX4_INFO("Excitation sequence complete");
			_phase = Phase::POST_SETTLE;
			_settle_time = 0.0f;
		}

		// Check for abort (RC released or disarmed)
		if (!_armed || trigger_falling_edge) {
			PX4_INFO("Excitation aborted");
			_generator.stop();
			_phase = Phase::IDLE;
			_manual_trigger = false;
		}

		break;

	case Phase::POST_SETTLE:
		// Wait for settle time after excitation
		_settle_time += dt;

		if (_settle_time >= SETTLE_DURATION_S) {
			_phase = Phase::IDLE;
			_manual_trigger = false;
			PX4_INFO("Post-settle complete");
		}

		break;
	}

	// Publish status
	publishStatus(excitation);

	perf_end(_loop_perf);
}

void MultisineExcitationModule::publishStatus(const float excitation[multisine::MAX_MOTORS])
{
	multisine_excitation_status_s status{};

	status.timestamp = hrt_absolute_time();
	status.active = (_phase == Phase::EXCITING);
	status.current_motor = _generator.getCurrentMotor();
	status.num_motors = _generator.getNumMotors();
	status.phase = static_cast<uint8_t>(_phase);
	status.elapsed_time = _generator.getElapsedTime();
	status.period_per_motor = _generator.getPeriodPerMotor();
	status.total_duration = _generator.getTotalDuration();

	// Copy excitation values, set unused to NaN
	for (size_t i = 0; i < multisine_excitation_status_s::NUM_EXCITATION; i++) {
		if (i < _generator.getNumMotors() && _phase == Phase::EXCITING) {
			status.excitation[i] = excitation[i];

		} else {
			status.excitation[i] = NAN;
		}
	}

	_status_pub.publish(status);
}

int MultisineExcitationModule::task_spawn(int argc, char *argv[])
{
	MultisineExcitationModule *instance = new MultisineExcitationModule();

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

int MultisineExcitationModule::runBenchTest(float throttle_override)
{
	PX4_WARN("================================================================");
	PX4_WARN("                      BENCH TEST MODE                          ");
	PX4_WARN("================================================================");
	PX4_WARN("WARNING: This will spin motors! Ensure propellers are REMOVED!");
	PX4_WARN("Press Enter to start, Ctrl+C to abort...");
	PX4_WARN("================================================================");

	// Wait for user confirmation
	char c;
	ssize_t ret = read(0, &c, 1);

	if (ret < 0) {
		PX4_ERR("Read failed: %i", errno);
		return PX4_ERROR;
	}

	// Load parameters efficiently
	int32_t num_motors = 8;
	float amplitude = 0.05f;
	float period = 15.0f;
	float freq_min = 0.1f;
	float freq_max = 1.0f;
	int32_t sequential = 1;
	float baseline_throttle = 0.15f;

	param_t handle = param_find("DTRG_MSINE_NMOT");
	if (handle != PARAM_INVALID) { param_get(handle, &num_motors); }

	handle = param_find("DTRG_MSINE_AMP");
	if (handle != PARAM_INVALID) { param_get(handle, &amplitude); }

	handle = param_find("DTRG_MSINE_T");
	if (handle != PARAM_INVALID) { param_get(handle, &period); }

	handle = param_find("DTRG_MSINE_FMIN");
	if (handle != PARAM_INVALID) { param_get(handle, &freq_min); }

	handle = param_find("DTRG_MSINE_FMAX");
	if (handle != PARAM_INVALID) { param_get(handle, &freq_max); }

	handle = param_find("DTRG_MSINE_SEQ");
	if (handle != PARAM_INVALID) { param_get(handle, &sequential); }

	handle = param_find("DTRG_MSINE_BTHR");
	if (handle != PARAM_INVALID) { param_get(handle, &baseline_throttle); }

	// Use override if specified
	if (throttle_override > 0.f) {
		baseline_throttle = throttle_override;
	}

	// Configure the generator (use dynamic allocation to reduce stack frame)
	multisine::MultisineExcitation *generator = new multisine::MultisineExcitation();

	if (!generator->configure(static_cast<uint8_t>(num_motors), period, freq_min, freq_max, amplitude)) {
		PX4_ERR("Failed to configure multisine generator");
		delete generator;
		return PX4_ERROR;
	}

	// Set sequential mode
	generator->setSequentialMode(sequential != 0);

	float total_duration = generator->getTotalDuration();

	PX4_INFO("Starting bench test:");
	PX4_INFO("  Motors: %d", (int)num_motors);
	PX4_INFO("  Baseline throttle: %.0f%%", (double)(baseline_throttle * 100.f));
	PX4_INFO("  Excitation amplitude: %.1f%%", (double)(amplitude * 100.f));
	PX4_INFO("  Total duration: %.1f s", (double)total_duration);
	PX4_INFO("  Sequential mode: %s", sequential ? "yes" : "no");
	if (sequential) {
		PX4_INFO("  Duration per motor: %.1f s", (double)(total_duration / num_motors));
	}
	PX4_INFO("");
	PX4_INFO("Press Enter to stop at any time...");

	// Create actuator test publisher
	uORB::Publication<actuator_test_s> actuator_test_pub{ORB_ID(actuator_test)};

	// Start the generator
	generator->start();

	// Calculate update interval (4ms = 250Hz)
	const uint32_t update_interval_us = 4000;
	const float dt_s = static_cast<float>(update_interval_us) / 1e6f;
	const uint32_t timeout_ms = 100;  // Keep alive timeout

	hrt_abstime last_print_time = hrt_absolute_time();
	bool running = true;

	// Set stdin to non-blocking
	int flags = fcntl(0, F_GETFL, 0);
	fcntl(0, F_SETFL, flags | O_NONBLOCK);

	while (running && generator->isActive()) {
		hrt_abstime now = hrt_absolute_time();

		// Check for user input to stop
		char input;

		if (read(0, &input, 1) > 0) {
			PX4_INFO("User requested stop");
			running = false;
			break;
		}

		// Update the generator with dt
		float excitation[multisine::MAX_MOTORS];
		generator->update(dt_s, excitation);

		// Apply baseline + excitation to each motor via actuator_test
		for (int motor = 0; motor < num_motors; motor++) {
			actuator_test_s msg{};
			msg.timestamp = now;
			msg.function = actuator_test_s::FUNCTION_MOTOR1 + motor;
			msg.value = baseline_throttle + excitation[motor];
			msg.action = actuator_test_s::ACTION_DO_CONTROL;
			msg.timeout_ms = timeout_ms;

			actuator_test_pub.publish(msg);
		}

		// Print progress every second
		if ((now - last_print_time) > 1000000) {
			float elapsed = generator->getElapsedTime();
			int current_motor = generator->getCurrentMotor();
			float progress = elapsed / total_duration * 100.f;
			PX4_INFO("Progress: %.1f%% | Motor: %d | Time: %.1f/%.1f s",
				 (double)progress, current_motor, (double)elapsed, (double)total_duration);
			last_print_time = now;
		}

		// Sleep until next update
		px4_usleep(update_interval_us);
	}

	// Restore stdin blocking mode
	fcntl(0, F_SETFL, flags);

	// Stop all motors by releasing control
	for (int motor = 0; motor < num_motors; motor++) {
		actuator_test_s msg{};
		msg.timestamp = hrt_absolute_time();
		msg.function = actuator_test_s::FUNCTION_MOTOR1 + motor;
		msg.value = NAN;
		msg.action = actuator_test_s::ACTION_RELEASE_CONTROL;
		msg.timeout_ms = 0;

		actuator_test_pub.publish(msg);
	}

	delete generator;
	PX4_INFO("Bench test completed");
	return PX4_OK;
}

int MultisineExcitationModule::runVerifyExcitation()
{
	PX4_INFO("Running excitation verification test...");

	// Load parameters efficiently
	int32_t num_motors = 4;
	float amplitude = 0.05f;

	param_t handle = param_find("DTRG_MSINE_NMOT");
	if (handle != PARAM_INVALID) { param_get(handle, &num_motors); }

	handle = param_find("DTRG_MSINE_AMP");
	if (handle != PARAM_INVALID) { param_get(handle, &amplitude); }

	// Use short period for quick test
	const float period = 2.0f;
	const float freq_min = 0.5f;
	const float freq_max = 2.0f;

	// Configure the generator in simultaneous mode (use dynamic allocation)
	multisine::MultisineExcitation *generator = new multisine::MultisineExcitation();

	if (!generator->configure(static_cast<uint8_t>(num_motors), period, freq_min, freq_max, amplitude)) {
		PX4_ERR("FAIL: Failed to configure multisine generator");
		delete generator;
		return PX4_ERROR;
	}

	generator->setSequentialMode(false);  // Simultaneous mode

	PX4_INFO("Test config: %d motors, period=%.1fs, freq=[%.1f-%.1f]Hz, amp=%.2f",
		 (int)num_motors, (double)period, (double)freq_min, (double)freq_max, (double)amplitude);

	// Start the generator
	generator->start();

	// Run for one full period
	const float test_duration = period;
	const uint32_t update_interval_us = 4000;  // 250Hz
	const float dt_s = static_cast<float>(update_interval_us) / 1e6f;

	// Track min/max for each motor (use dynamic arrays to save stack space)
	float *min_val = new float[num_motors];
	float *max_val = new float[num_motors];
	float *sum_val = new float[num_motors];
	int sample_count = 0;

	for (int i = 0; i < (int)num_motors; i++) {
		min_val[i] = 1.0f;
		max_val[i] = -1.0f;
		sum_val[i] = 0.0f;
	}

	hrt_abstime start_time = hrt_absolute_time();

	while (generator->isActive()) {
		float elapsed = static_cast<float>(hrt_absolute_time() - start_time) / 1e6f;

		if (elapsed > test_duration) {
			break;
		}

		float excitation[multisine::MAX_MOTORS];
		generator->update(dt_s, excitation);

		for (int motor = 0; motor < (int)num_motors; motor++) {
			if (excitation[motor] < min_val[motor]) {
				min_val[motor] = excitation[motor];
			}

			if (excitation[motor] > max_val[motor]) {
				max_val[motor] = excitation[motor];
			}

			sum_val[motor] += excitation[motor];
		}

		sample_count++;
		px4_usleep(update_interval_us);
	}

	generator->stop();

	// Verify results
	PX4_INFO("Verification results (%d samples):", sample_count);

	bool all_passed = true;
	const float min_expected_range = amplitude * 0.5f;  // Expect at least 50% of amplitude as range

	for (int motor = 0; motor < (int)num_motors; motor++) {
		float range = max_val[motor] - min_val[motor];
		float mean = sum_val[motor] / static_cast<float>(sample_count);
		bool passed = (range > min_expected_range);

		PX4_INFO("  Motor %d: range=%.4f, min=%.4f, max=%.4f, mean=%.4f %s",
			 motor, (double)range, (double)min_val[motor], (double)max_val[motor],
			 (double)mean, passed ? "[PASS]" : "[FAIL]");

		if (!passed) {
			all_passed = false;
		}
	}

	// Cleanup dynamic arrays
	delete[] min_val;
	delete[] max_val;
	delete[] sum_val;
	delete generator;

	if (all_passed) {
		PX4_INFO("VERIFICATION PASSED: All motors show proper excitation");
		return PX4_OK;

	} else {
		PX4_ERR("VERIFICATION FAILED: Some motors not showing proper excitation");
		return PX4_ERROR;
	}
}

int MultisineExcitationModule::print_status()
{
	PX4_INFO("Multisine Excitation Module");
	PX4_INFO("  Enabled: %s", _param_enable.get() ? "yes" : "no");
	PX4_INFO("  Armed: %s", _armed ? "yes" : "no");
	PX4_INFO("  Phase: %s",
		 _phase == Phase::IDLE ? "IDLE" :
		 _phase == Phase::PRE_SETTLE ? "PRE_SETTLE" :
		 _phase == Phase::EXCITING ? "EXCITING" : "POST_SETTLE");
	PX4_INFO("  Generator configured: %s", _generator.isConfigured() ? "yes" : "no");
	PX4_INFO("  Sequential mode: %s", _generator.getSequentialMode() ? "yes" : "no");

	if (_generator.isConfigured()) {
		PX4_INFO("  Configuration:");
		PX4_INFO("    Motors: %d", _generator.getNumMotors());
		PX4_INFO("    Period per motor: %.1f s", (double)_generator.getPeriodPerMotor());
		PX4_INFO("    Total duration: %.1f s", (double)_param_period.get());
		PX4_INFO("    Freq range: %.2f - %.2f Hz", (double)_param_freq_min.get(), (double)_param_freq_max.get());
		PX4_INFO("    Amplitude: %.3f", (double)_param_amplitude.get());
	}

	if (_generator.isActive()) {
		PX4_INFO("  Active excitation:");
		PX4_INFO("    Current motor: %d", _generator.getCurrentMotor());
		PX4_INFO("    Elapsed time: %.2f s", (double)_generator.getElapsedTime());
	}

	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);

	return 0;
}

int MultisineExcitationModule::custom_command(int argc, char *argv[])
{
	if (argc < 1) {
		return print_usage("missing command");
	}

	if (!strcmp(argv[0], "start_excitation")) {
		MultisineExcitationModule *instance = get_instance();

		if (instance) {
			if (!instance->_armed) {
				PX4_WARN("Cannot start: not armed");
				return PX4_ERROR;
			}

			if (instance->_phase != Phase::IDLE) {
				PX4_WARN("Excitation already in progress");
				return PX4_ERROR;
			}

			PX4_INFO("Manual trigger: starting excitation");
			instance->_manual_trigger = true;
			return PX4_OK;

		} else {
			PX4_ERR("Module not running");
			return PX4_ERROR;
		}
	}

	if (!strcmp(argv[0], "stop_excitation")) {
		MultisineExcitationModule *instance = get_instance();

		if (instance) {
			if (instance->_phase != Phase::IDLE) {
				PX4_INFO("Stopping excitation");
				instance->_generator.stop();
				instance->_phase = Phase::IDLE;
				instance->_manual_trigger = false;
			} else {
				PX4_INFO("No excitation in progress");
			}

			return PX4_OK;

		} else {
			PX4_ERR("Module not running");
			return PX4_ERROR;
		}
	}

	if (!strcmp(argv[0], "help")) {
		return print_usage();
	}

	if (!strcmp(argv[0], "bench_test")) {
		// Parse optional throttle override from command line
		float throttle_override = -1.f;

		if (argc >= 2) {
			throttle_override = strtof(argv[1], nullptr);

			if (throttle_override < 0.05f || throttle_override > 0.5f) {
				PX4_ERR("Throttle must be between 0.05 and 0.5");
				return PX4_ERROR;
			}
		}

		return runBenchTest(throttle_override);
	}

	if (!strcmp(argv[0], "verify_excitation")) {
		// Non-interactive verification test for SITL
		// Runs for a short time and verifies excitation signals are generated
		return runVerifyExcitation();
	}

	return print_usage("unknown command");
}

int MultisineExcitationModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Orthogonal Multisine Excitation Module for RLS System Identification.

Generates orthogonal multisine signals based on Morelli's method for
identifying the control effectiveness matrix (B matrix) of multirotor
aircraft.

Features:
- Orthogonal frequency assignment using harmonic combing
- Schroeder phase optimization for low crest factor
- Sequential or simultaneous motor excitation
- RC aux switch trigger or manual command trigger
- Safety: only active when armed, auto-stops on disarm

The excitation is applied by the control_allocator module.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("multisine_excitation", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	PRINT_MODULE_USAGE_COMMAND_DESCR("start_excitation", "Manually trigger excitation (must be armed)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop_excitation", "Manually stop excitation");
	PRINT_MODULE_USAGE_COMMAND_DESCR("bench_test", "Run motors at low throttle with excitation (REMOVE PROPS!)");
	PRINT_MODULE_USAGE_ARG("<throttle>", "Optional baseline throttle (0.05-0.5), default from DTRG_MSINE_BTHR", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("verify_excitation", "Run automated excitation verification (for SITL testing)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("help", "Print help/usage info");

	return 0;
}

extern "C" __EXPORT int multisine_excitation_main(int argc, char *argv[])
{
	return MultisineExcitationModule::main(argc, argv);
}

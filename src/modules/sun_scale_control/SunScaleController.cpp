/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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

#include "SunScaleController.hpp"


using namespace time_literals;


SunScaleController::SunScaleController():
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_valid_hysteresis.set_hysteresis_time_from(false, 2_s);
	updateParams();
	// PX4_INFO("GG");
}

SunScaleController::~SunScaleController()
{
	// perf_free(_cycle_perf);
}


bool SunScaleController::init()
{
	// execute Run() on every rls_wrench_estimator publication
	if (!_rc_channels1_sub.registerCallback()) {
		PX4_ERR("local position callback registration failed");
		return false;
	}

	//limit to 100Hz
	_rc_channels1_sub.set_interval_us(10_ms);

	return true;
}


void SunScaleController::updateParams()
{
	ModuleParams::updateParams();
	// sunScale=1;
	sunScale=_param_mixer_sun_scale.get();
	sunConst=_param_mixer_sun_const.get();
	sunStick=_param_mixer_sun_stick.get()-1;

}

int SunScaleController::task_spawn(int argc, char *argv[])
{
	SunScaleController *instance = new SunScaleController();

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

int SunScaleController::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SunScaleController::print_usage(const char *reason)
{
	PRINT_MODULE_DESCRIPTION("GG");
	return 0;
}


void SunScaleController::Run()
{
	if (should_exit()) {
		_rc_channels1_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// perf_begin(_loop_perf);

	// Push backup schedule
	ScheduleDelayed(50_ms);


	if (_local_pos_sp_sub.update(&local_pos_sp) && _local_pos_sub.update(&local_pos) ) {
		if  (!(isnan(local_pos_sp.x)  || isnan(local_pos_sp.y) || isnan(local_pos_sp.z))){
			error(counter)=powf(local_pos_sp.x-local_pos.x,2)+powf(local_pos_sp.y-local_pos.y,2);
			timestamps(counter)=local_pos.timestamp;

			float value=0;
			for (int i=0;i<100;i++){
				value+=error(i);
			}



			error_rms_s error_rms{};

			error_rms.timestamp = hrt_absolute_time();
			error_rms.end_read=timestamps(counter);
			error_rms.start_read=timestamps(end);
			// for (int i=0;i<100;i++){
			// 	error_rms.timestamps[i]=timestamps(i);
			// }
			error_rms.error_rms=value;

			counter++;
			if(counter>=100){
				counter=0;
			}
			end++;

			if(end>=100){
				end=0;
			}
			error_rms.goal=0;
			if (_debug_vect_sub.updated()) {
				debug_vect_s flags_vect;
				_debug_vect_sub.copy(&flags_vect);
				error_rms.goal = (flags_vect.z);
			}
			_error_rms_pub.publish(error_rms);
		}
	}





	if(_rc_channels_sub.update(&_rc_channels)){

		float newSunScale;

		if(!sunConst){
			float minOut=0.5;
			float maxOut=2;

			float minIn=-1;
			float maxIn=1;

			float sunStickVal=_rc_channels.channels[sunStick];


			newSunScale=(((sunStickVal-minIn)/(maxIn-minIn))*(maxOut-minOut))+minOut;

			newSunScale=math::max(math::min(newSunScale,maxOut),minOut);




			// PX4_INFO("%f",double(newSunScale));
		}else{
			newSunScale=sunScale;
		}


		control_sun_scale_s control_sun_scale{};

		control_sun_scale.timestamp = hrt_absolute_time();
		control_sun_scale.sun_scale = newSunScale;

 		_control_sun_scale_pub.publish(control_sun_scale);
	}

}

extern "C" __EXPORT int sun_scale_control_main(int argc, char *argv[])
{
	return SunScaleController::main(argc, argv);
}

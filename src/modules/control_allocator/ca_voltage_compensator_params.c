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
 * @file ca_voltage_compensator_params.c
 *
 * CA-specific voltage compensator parameter (CA_VC_EN).
 * Battery model coefficients are shared with the bench_test module
 * and defined in the "Voltage Compensator" param group (VC_* prefix).
 */

/**
 * Voltage compensator enable
 *
 * Enable the battery-voltage compensator in the control allocator.
 * When enabled, motor commands are scaled to keep effective motor voltage
 * constant regardless of battery sag. Requires CA_VC_VBOP > 0.
 *
 * 0 = disabled
 * 1 = predicted mode (full battery model: estimates ω, current, RC-branch
 *     voltage drop – compensates based on predicted terminal voltage)
 * 2 = simple mode (reads the instantaneous measured terminal voltage from
 *     battery_status and compensates directly – no model integration)
 *
 * @value 0 Disabled
 * @value 1 Predicted (battery model)
 * @value 2 Simple (measured voltage)
 * @group Control Allocator
 * @reboot_required true
 */
PARAM_DEFINE_INT32(CA_VC_EN, 0);

/* Model coefficients shared with bench_test — see VC_* param group (Voltage Compensator) */

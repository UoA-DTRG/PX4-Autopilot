/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file mc_att_control_params.c
 * Parameters for multicopter attitude controller.
 *
 * @author Salim Al-zubaidi
 */

/**
 * Control Sun Scaling
 *
 * Setting the Sun Motor Map Scale.
 * 1 is equal usage. More than 1 mean more usage of sun rotor
 *
 * @min 0.5
 * @max 2.0
 * @decimal 1
 * @group Variable Motor Map
 */
PARAM_DEFINE_FLOAT(MIXER_SUN_SCALE, 1.0f);

/**
 * Constant Sun Scaling
 *
 * Setting constant Sun Motor Map Scaling.
 * 0 mean
 *
 * @boolean
 * @group Variable Motor Map
 */
PARAM_DEFINE_INT32(MIXER_SUN_CONST, 1);

/**
 * DTRG Horizontal Thrust Switch
 *
 * Define which aux channel will be used for the horizontal thrust control in the X direction. (default is channel 8)
 *
 * WARNING - ensure that the selected channel is not used for any other function and that the channel is correctly configured in the radio, typically use channels 8 -13 for this function as they are unlikely to be used for other functions.
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @reboot_required true
 * @group Variable Motor Map
 */
PARAM_DEFINE_INT32(MIXER_SUN_STICK, 6);

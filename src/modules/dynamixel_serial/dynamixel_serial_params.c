/****************************************************************************
 *
 *   Copyright (c) 2015-2016 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file dynamixel_serial_params.c
 * Parameter definition for dynamixel_serial.
 *
 * @author Pedro Mendes <pmen817@aucklanduni.ac.nz>
 *
 */

/**
 * Position Minimum - Dynamixel
 *
 * Minimum desired position for Position Control Mode(3) within the range of 1 rotation.
 * Goal Position(116) should be configured within the position limit range.
 *
 * @group Dynamixel
 * @min 0
 * @max 4095
 */
PARAM_DEFINE_INT32(DYN_SER_POS_MIN, 0);

/**
 * Position Maximum - Dynamixel
 *
 * Maximum desired position for Position Control Mode(3) within the range of 1 rotation.
 * Goal Position(116) should be configured within the position limit range.
 *
 * @group Dynamixel
 * @min 0
 * @max 4095
 */
PARAM_DEFINE_INT32(DYN_SER_POS_MAX, 4095);

/**
 * Velocity Maximum - Dynamixel
 *
 * Maximum desired velocity for Goal Velocity(104) in Velocity Control Mode(1).
 *
 * @group Dynamixel
 * @min 0
 * @max 285
 */
PARAM_DEFINE_INT32(DYN_SER_VEL_MAX, 285);

/**
 * Robotis servo position max
 *
 * Position maximum at servo max value. This should be within the position control range of the servos, normally 0 to 4095
 *
 * @group Dynamixel
 * @min 0
 * @max 1941
 */
PARAM_DEFINE_INT32(DYN_SER_CUR_MAX, 1941);

/**
 * Robotis servo position max
 *
 * Position maximum at servo max value. This should be within the position control range of the servos, normally 0 to 4095
 *
 * @group Dynamixel
 * @min 0
 * @max 1048575
 */
PARAM_DEFINE_INT32(DYN_SER_EXT_MAX, 1048575);

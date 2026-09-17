uint64 timestamp        # time since system start (microseconds)

float32[3] fe           # estimated external force  (body frame) [N]
float32[3] me           # estimated external moment (body frame) [N.m]

float32[3] fi           # actuator force vector  (body frame) [N]
float32[3] mi           # actuator moment vector (body frame) [N.m]

float32[16] k_f         # estimated thrust coefficient of each motor (unused slots = NAN)
float32[3] x_offset     # estimated center-of-mass offset [x, y, z] [m]

uint8 n_groups          # number of active thrust-coefficient groups
uint8[16] motor_group   # thrust-coefficient group index for each motor

uint8 STAGE_IDENTIFY = 0        # k_f and CoM offset adapting
uint8 STAGE_ALIGN = 1           # identification frozen, sensor alignment being estimated
uint8 STAGE_INTERACT = 2        # everything frozen, fe/me are the measurement

uint8 stage             # identification stage, see STAGE_*
float32[2] alignment    # sensor-to-rotor misalignment being estimated [rad] (STAGE_ALIGN)
float32[2] align_force  # world-fixed force separated from it, NED [N] (STAGE_ALIGN)
float32 align_coverage  # heading span visited during STAGE_ALIGN [deg]

bool interaction_flag   # in STAGE_INTERACT
bool valid              # estimation valid

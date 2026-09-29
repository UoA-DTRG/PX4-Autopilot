uint64 timestamp        # time since system start (microseconds)

float32[3] fe           # estimated external force  (body frame) [N]
float32[3] me           # estimated external moment (body frame) [N.m]

float32[3] fi           # actuator force vector  (body frame) [N]
float32[3] mi           # actuator moment vector (body frame) [N.m]

float32[16] k_f         # estimated thrust coefficient of each motor (unused slots = NAN)
float32[3] x_offset     # estimated center-of-mass offset [x, y, z] [m]

uint8 n_groups          # number of active thrust-coefficient groups
uint8[16] motor_group   # thrust-coefficient group index for each motor

bool interaction_flag   # external interaction detected (RLS gains frozen)
bool valid              # estimation valid

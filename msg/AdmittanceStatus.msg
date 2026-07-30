# Admittance controller status and telemetry.
# Published every cycle whether or not the controller is engaged, so that the
# expected admittance response can be logged while the module is bypassed.

uint64 timestamp                # time since system start (microseconds)

uint8 BYPASS_NONE = 0           # not bypassed, output is being applied
uint8 BYPASS_DISABLED = 1       # ADM_CTR_EN != 2
uint8 BYPASS_RC_SWITCH = 2      # RC bypass switch (ADM_CTR_RC_CH) is low
uint8 BYPASS_NAV_STATE = 3      # current nav_state not selected in ADM_CTR_NAV
uint8 BYPASS_DISARMED = 4       # vehicle not armed
uint8 BYPASS_WRENCH_INVALID = 5 # rls_wrench_estimator reported invalid
uint8 BYPASS_INPUT_INVALID = 6  # non-finite input, or dt out of range
uint8 BYPASS_OUTPUT_INVALID = 7 # computed setpoint was non-finite

bool engaged                    # true = mc_pos_control should use admittance_setpoint
bool valid                      # admittance state is initialised and integrating
uint8 bypass_reason             # BYPASS_*

float32 ramp                    # engage ramp applied to the deviation, 0..1

float32[4] wrench_used          # deadzoned/saturated wrench fed to the ODE [fx fy fz mz], heading frame
float32[4] mass                 # virtual mass  M per axis [x y z yaw]
float32[4] damping              # virtual damping C per axis
float32[4] stiffness            # virtual stiffness K per axis

float32[3] deviation            # admittance deviation from the reference, NED [m]
float32[3] deviation_rate       # deviation rate, NED [m/s]
float32 yaw_deviation           # yaw deviation [rad]

float32[3] reference_position   # internal reference position, NED [m]
bool reference_integrating      # true when the reference is being integrated from velocity

float32 sat_factor              # filtered actuator saturation factor, 0..1
float32 target_dist             # target distance used for the bell-curve exponent [m]

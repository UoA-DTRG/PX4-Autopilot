# Multisine excitation status for system identification
# Published by the multisine_excitation module
# Consumed by control_allocator to add excitation to motor outputs

uint64 timestamp                        # time since system start (microseconds)

bool active                             # true if excitation is currently active
uint8 current_motor                     # current motor being excited (0-indexed)
uint8 num_motors                        # total number of motors configured
uint8 phase                             # current phase: 0=idle, 1=pre-settle, 2=exciting, 3=post-settle
float32 elapsed_time                    # elapsed time in current excitation (seconds)
float32 period_per_motor                # configured period per motor (seconds)
float32 total_duration                  # total excitation duration (seconds)

uint8 NUM_EXCITATION = 16               # max number of motor excitation values

float32[16] excitation                  # excitation values for each motor [-1, 1], NaN if unused

# Voltage compensator status for bench_test module.
# Published once per target-motor command when -c is active.
# Arrays are indexed [0..7] by motor slot; unused slots are zero.
# delta_comp / c_delta are only meaningful for the target motor (motor_index);
# all other slots carry the raw command actually sent to that motor.

uint64 timestamp          # time since system start (microseconds)

uint8  motor_index        # 0-based index of the motor being compensated this cycle
uint8  n_motors           # number of active motors

float32[8] delta_raw      # raw normalised commands per motor [0, 1]
float32[8] delta_comp     # compensated command for target, raw for others [0, 1]
float32[8] c_delta        # correction factor (delta_comp/delta_raw); 1.0 for non-targets

float32 soc               # battery state of charge [0, 1], -1 if unavailable
float32 v_b_pred          # predicted terminal battery voltage (V)
float32 i_total           # predicted total current draw (A)

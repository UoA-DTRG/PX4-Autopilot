# Voltage compensator status for bench_test module
# Published by bench_test on every compensatedCommandMotor() call when -c is active.
# Logs the raw and compensated motor commands together with the battery state
# used to compute the correction, providing a full audit trail of the compensator.

uint64 timestamp        # time since system start (microseconds)

uint8  motor_index      # 0-based motor index this command was issued for
float32 delta_raw       # raw normalised motor command sent by the test [0, 1]
float32 delta_comp      # compensated command actually sent to the motor [0, 1]
float32 C_delta         # correction factor: delta_comp / delta_raw
float32 soc             # battery state of charge used [0, 1], -1 if unavailable
float32 V_b_pred        # predicted terminal battery voltage (V)
float32 I_total         # predicted total current draw (A)

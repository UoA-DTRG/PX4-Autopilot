# Voltage compensator status — published by bench_test (per-motor stepping)
# and control_allocator (in-flight, all motors).
# Arrays are indexed [0..7] by motor slot; unused slots are zero.

uint64 timestamp          # time since system start (microseconds)

uint8  motor_index        # 0-based target motor (0xFF = all motors, in-flight)
uint8  n_motors           # number of active motors

# ── Per-motor commands ───────────────────────────────────────────────
float32[8] delta_raw      # raw normalised commands per motor [0, 1]
float32[8] delta_comp     # compensated commands per motor [0, 1]
float32[8] c_delta        # correction factor per motor (delta_comp/delta_raw)

# ── Per-motor predicted states ───────────────────────────────────────
float32[8] omega          # predicted rotor speed per motor (rad/s, scaled)
float32[8] i_motor        # predicted current draw per motor (A)

# ── Battery model scalars ────────────────────────────────────────────
float32 soc               # battery state of charge [0, 1], -1 if unavailable
float32 v_b_pred          # predicted terminal battery voltage (V)
float32 i_total           # predicted total current draw (A)
float32 v0                # open-circuit voltage V0(SoC) (V)
float32 r0                # series resistance R0(SoC) (Ohm)
float32 r1                # RC-branch resistance R1(SoC) (Ohm)
float32 tau1              # RC-branch time constant τ1(SoC) (s)
float32 v_rc              # RC-branch voltage state (V)

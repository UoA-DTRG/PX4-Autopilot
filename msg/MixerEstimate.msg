uint64 timestamp                        # time since system start (microseconds)

uint8 num_rotors                        # Number of rotors in the configuration

# Mixer matrix (6 x num_rotors)
# Pseudo-inverse of effectiveness matrix
# Maps control inputs [Fx, Fy, Fz, Mx, My, Mz] to actuator commands
# Row-major order: [fx_to_rotor0, fx_to_rotor1, ..., mz_to_rotor15]
float32[96] mixer_matrix                # Maximum 6 DOF x 16 rotors = 96 elements

bool mixer_valid                        # True if mixer is valid and usable

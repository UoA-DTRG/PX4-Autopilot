uint64 timestamp                        # time since system start (microseconds)

uint8 num_rotors                        # Number of rotors in the configuration

# Mixer matrix (num_rotors x 6)
# Pseudo-inverse of effectiveness matrix
# Maps control inputs [Fx, Fy, Fz, Mx, My, Mz] to actuator commands
# Row-major storage: [fx_to_rotor0, fy_to_rotor0, ..., mz_to_rotor0, fx_to_rotor1, ...]
float32[96] mixer_matrix                # Maximum 16 rotors x 6 DOF = 96 elements

bool mixer_valid                        # True if mixer is valid and usable

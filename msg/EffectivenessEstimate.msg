uint64 timestamp                        # time since system start (microseconds)

uint8 num_rotors                        # Number of rotors in the configuration

# Effectiveness matrix (num_rotors x 6)
# Each rotor contributes to [Fx, Fy, Fz, Mx, My, Mz]
# Row-major order: [rotor0_fx, rotor0_fy, ..., rotor0_mz, rotor1_fx, ...]
float32[96] effectiveness_matrix        # Maximum 16 rotors x 6 DOF = 96 elements

# Estimation quality metrics
float32 estimation_variance             # Variance of the estimation
float32 innovation                      # Innovation (prediction error)
bool estimation_valid                   # True if estimation is valid and converged

# Physical parameters (from RLS estimation)
float32 mass                            # Estimated vehicle mass (kg)
float32[3] inertia_diagonal             # Estimated inertia diagonal [Ixx, Iyy, Izz] (kg*m^2)

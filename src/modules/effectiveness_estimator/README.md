# Effectiveness Estimator Module

## Overview

The Effectiveness Estimator is an optional PX4 module that estimates the actuator effectiveness matrix and mixer matrix for multicopter vehicles using Recursive Least Squares (RLS) estimation.

## Purpose

This module is designed for:
- System identification and parameter estimation
- Validation of control allocation effectiveness
- Research and development of adaptive control strategies
- Logging effectiveness estimates for offline analysis

**Note:** This module is currently a framework for RLS-based estimation. The estimates are logged for validation but NOT used in active control allocation.

## Features

- **RLS Parameter Estimation**: Framework for recursive least squares estimation
- **Configurable Parameters**: Extensive parameter set for tuning the estimator
- **Optional Module**: Disabled by default, no impact on standard PX4 operation
- **Logging**: Publishes effectiveness and mixer estimates via uORB topics
- **Angular Acceleration**: Computed via finite difference from angular velocity

## Configuration Parameters

### Enable/Disable
- `EFF_EST_ENABLE` (INT32): Enable/disable the estimator (default: 0/disabled)

### RLS Parameters
- `EFF_EST_LAMBDA` (FLOAT): Forgetting factor (0.9 - 0.9999, default: 0.99)
- `EFF_EST_P_INIT` (FLOAT): Initial covariance diagonal value (1.0 - 10000.0, default: 1000.0)

### Vehicle Parameters
- `EFF_EST_NUM_ROTORS` (INT32): Number of rotors/motors (1 - 16, default: 4)
- `EFF_EST_MASS` (FLOAT): Vehicle mass in kg (0.1 - 100.0, default: 1.5)
- `EFF_EST_IXX` (FLOAT): Moment of inertia about X axis in kg*m² (default: 0.029)
- `EFF_EST_IYY` (FLOAT): Moment of inertia about Y axis in kg*m² (default: 0.029)
- `EFF_EST_IZZ` (FLOAT): Moment of inertia about Z axis in kg*m² (default: 0.055)

### Update Rate
- `EFF_EST_UPDATE_RATE` (FLOAT): Estimator update frequency in Hz (10.0 - 500.0, default: 50.0)

### Convergence Parameters
- `EFF_EST_CONV_VAR` (FLOAT): Maximum average variance for convergence (0.01 - 10.0, default: 1.0)
- `EFF_EST_CONV_INNOV` (FLOAT): Maximum RMS innovation for convergence in N·m (0.1 - 100.0, default: 10.0)
- `EFF_EST_MIN_EXCITE` (FLOAT): Minimum actuator excitation threshold (0.001 - 0.5, default: 0.01)

## Published Topics

### effectiveness_estimate
Contains the estimated effectiveness matrix:
- `num_rotors`: Number of rotors
- `effectiveness_matrix`: 6 x num_rotors matrix (row-major format)
  - Maps actuator commands to forces/moments
  - Rows: Fx, Fy, Fz, Mx, My, Mz
- `estimation_valid`: Indicates if estimation has converged
- `mass`, `inertia_diagonal`: Vehicle physical parameters

### mixer_estimate
Contains the computed mixer matrix:
- `num_rotors`: Number of rotors
- `mixer_matrix`: num_rotors x 6 matrix (row-major format)
  - Pseudo-inverse of effectiveness matrix
  - Maps control inputs to actuator commands
- `mixer_valid`: Indicates if mixer is valid

## Usage

### Enable the Module
```
param set EFF_EST_ENABLE 1
param set EFF_EST_NUM_ROTORS 4  # Set to your vehicle's rotor count
```

### Configure Vehicle Parameters
```
param set EFF_EST_MASS 1.5      # Vehicle mass in kg
param set EFF_EST_IXX 0.029     # Inertia about X axis
param set EFF_EST_IYY 0.029     # Inertia about Y axis
param set EFF_EST_IZZ 0.055     # Inertia about Z axis
```

### Configure Convergence Thresholds (Optional)
```
param set EFF_EST_CONV_VAR 1.0      # Maximum variance for convergence
param set EFF_EST_CONV_INNOV 10.0   # Maximum RMS innovation (N*m)
param set EFF_EST_MIN_EXCITE 0.01   # Minimum actuator excitation
```

### Start the Module
The module will start automatically when armed if `EFF_EST_ENABLE` is set to 1.

Alternatively, start manually:
```
effectiveness_estimator start
```

### View Status
```
effectiveness_estimator status
```

### Stop the Module
```
effectiveness_estimator stop
```

## Logging

The effectiveness and mixer estimates are published to uORB topics and can be logged using the PX4 logger. Add the following to your logging profile:

```
effectiveness_estimate
mixer_estimate
```

## Implementation Status

### Current State
- ✅ Module structure and framework
- ✅ Message definitions
- ✅ Parameter configuration (12 parameters)
- ✅ Angular acceleration computation
- ✅ **Complete RLS estimation algorithm**
- ✅ **Convergence detection with configurable thresholds**
- ✅ **Mixer pseudo-inverse computation**
- ✅ Logger integration
- ✅ Board configuration (SITL default)

### RLS Algorithm Details

The implemented RLS algorithm:
1. **Regressor Construction**: Forms regressor vector from actuator outputs
2. **Measurement Vector**: Computes measured moments = angular_acceleration × inertia
3. **Kalman Gain**: K = P*phi / (lambda + phi'*P*phi)
4. **Innovation**: innovation = measurement - phi'*theta
5. **Parameter Update**: theta = theta + K*innovation
6. **Covariance Update**: P = (P - K*phi'*P) / lambda
7. **Per-Axis Tracking**: Separate innovation tracking for Mx, My, Mz

### Convergence Criteria

Estimation is marked valid when:
- Sample count ≥ 100
- Average parameter variance < EFF_EST_CONV_VAR (default: 1.0)
- RMS innovation < EFF_EST_CONV_INNOV (default: 10.0 N·m)

### Future Enhancements (Optional)
1. Force axis estimation (Fx, Fy, Fz) in addition to moments
2. Online inertia estimation
3. Adaptive forgetting factor
4. Integration with control allocator (research feature)

## Technical Details

### Matrix Storage Format

**Effectiveness Matrix** (6 x num_rotors):
- Row-major storage: `[rotor0_fx, rotor1_fx, ..., rotor15_fx, rotor0_fy, ...]`
- Row i = contribution of all rotors to force/moment i
- Rows: 0=Fx, 1=Fy, 2=Fz, 3=Mx, 4=My, 5=Mz

**Mixer Matrix** (num_rotors x 6):
- Row-major storage: `[fx_to_rotor0, fy_to_rotor0, ..., mz_to_rotor0, fx_to_rotor1, ...]`
- Row i = how rotor i responds to each force/moment command

### Dependencies
- Matrix library (`lib/matrix`)
- MathLib (`mathlib`)
- uORB messaging system
- PX4 work queue

## References

- PX4 Module Structure: Similar pattern to `mc_hover_thrust_estimator`
- RLS Algorithm: See `src/lib/system_identification/arx_rls.hpp`
- Control Allocation: See `src/modules/control_allocator`

## License

BSD 3-Clause License (see file headers)

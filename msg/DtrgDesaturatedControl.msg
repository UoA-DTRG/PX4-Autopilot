# DTRG sequential desaturation (MC_AIRMODE = 0): the control command the allocator actually
# mixed, after desaturation, next to the demand it was given. Same units as
# vehicle_torque_setpoint / vehicle_thrust_setpoint (normalised).
# Published by ControlAllocationSequentialDesaturation on every allocation.
# The difference torque - torque_sp (thrust - thrust_sp) is the gain in sequential_desaturation.
# Actuator slew rate limiting and clipping, applied after this, are not included.
uint64 timestamp		# time since system start (microseconds)

float32[3] torque_sp		# demanded roll, pitch, yaw torque (before desaturation)
float32[3] thrust_sp		# demanded thrust X, Y, Z (before desaturation)

float32[3] torque		# roll, pitch, yaw torque after desaturation
float32[3] thrust		# thrust X, Y, Z after desaturation

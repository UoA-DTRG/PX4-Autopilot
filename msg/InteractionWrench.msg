# Wrench measured at the interaction rod mount by the simulator's force_torque
# sensor, for physical-interaction experiments. SITL only: published by
# gz_bridge when the vehicle model carries an interaction rod, and never on
# real hardware.
# implementation: UoA DTRG

uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample		# timestamp of the raw simulator sample (microseconds)

# Wrench the rod transmits into the airframe, in body FRD axes. Pressing the
# rod forward into a surface gives a negative force[0]: the reported quantity
# is what the environment feeds back into the vehicle, not what the vehicle
# exerts. Includes the rod's own weight and its inertial reaction, so it is the
# external contact force only near steady flight.
float32[3] force		# [N]
float32[3] torque		# [N.m]

##This messgae HorizontalThrustLimit.msg publishes the state of the horizontal thrust limit
#implemention: 	UoA DTRG

uint64 	timestamp		# time since system start (microseconds)

# horizontal thrust limit flags
uint8 x_sat
uint8 y_sat

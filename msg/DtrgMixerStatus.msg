# DTRG CSV mixer (DTRG_MIXER_CSV): result of the last attempt to load the mixer file.
# Published by control_allocator, read by the commander arming check, which refuses
# to arm unless the status is STATUS_LOADED (or STATUS_DISABLED).
uint64 timestamp			# time since system start (microseconds)

uint8 STATUS_DISABLED = 0		# DTRG_MIXER_CSV is off, the mixer is the pseudo-inverse of the geometry
uint8 STATUS_LOADED = 1			# mixer file loaded
uint8 STATUS_FILE_NOT_FOUND = 2		# the file cannot be opened (missing, or no SD card)
uint8 STATUS_EMPTY = 3			# the file holds no rows
uint8 STATUS_SHORT_ROW = 4		# a row has fewer values than axes (roll, pitch, yaw, x, y, z)
uint8 STATUS_INVALID_VALUE = 5		# a value is not a finite number (text, nan, inf, too long)
uint8 STATUS_ROW_COUNT_MISMATCH = 6	# the number of rows differs from the number of configured actuators
uint8 STATUS_ALL_ZERO = 7		# every coefficient is 0

uint8 status
uint16 line				# line of the file with the error (STATUS_SHORT_ROW, STATUS_INVALID_VALUE), 1-based
uint8 num_rows				# rows read from the file
uint8 num_actuators			# actuators configured

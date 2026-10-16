# DTRG constant command (planetaryHex): the rotors in DTRG_CC_MOTORS (by default the centre
# rotors, Motors 7 and 8) held at a fixed command (DTRG_CC_M_CMD) while the other rotors are allocated the controller's
# demand minus what the centre rotors produce. Published by control_allocator while DTRG_CC_EN
# is set: every allocation while held or ramping, otherwise on change and at 1 Hz.
uint64 timestamp		# time since system start (microseconds)

bool requested			# DTRG_CC_EN set, switch (RC_MAP_CC_MODE) on, RC present, armed and flying
bool available			# multirotor geometry, no CSV mixer, valid DTRG_CC_MOTORS (1 to number of motors - 6), no handled motor failure
float32 blend			# 0: normal allocation, 1: centre rotors at DTRG_CC_M_CMD, ramped over DTRG_CC_RAMP
uint16 motors			# held motors (latched DTRG_CC_MOTORS), bit 0: Motor 1
float32[6] cc_cmd		# commands of the held motors in ascending order while blend > 0, NAN otherwise

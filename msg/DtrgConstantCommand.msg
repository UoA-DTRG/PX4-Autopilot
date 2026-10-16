# DTRG constant command (planetaryHex): the centre rotors, Motors 7 and 8, held at fixed
# commands (DTRG_CC_M_CMD) while the six outer rotors are allocated the controller's
# demand minus what the centre rotors produce. Published by control_allocator while DTRG_CC_EN
# is set: every allocation while held or ramping, otherwise on change and at 1 Hz.
uint64 timestamp		# time since system start (microseconds)

bool requested			# DTRG_CC_EN set, switch (RC_MAP_CC_MODE) on, RC present, armed and flying
bool available			# multirotor geometry, no CSV mixer, at least 8 motors, no handled motor failure
float32 blend			# 0: normal allocation, 1: centre rotors at DTRG_CC_M_CMD, ramped over DTRG_CC_RAMP
float32[2] cc_cmd		# commands of Motors 7 and 8 while blend > 0, NAN otherwise

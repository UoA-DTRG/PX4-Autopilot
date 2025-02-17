/**
 * Enable load cell driver
 *
 * @boolean
 * @reboot_required true
 * @group Sensors
 */
PARAM_DEFINE_INT32(LOADCELL_EN, 0);

/**
 * Load cell UART port
 *
 * @value 0 DISABLED
 * @value 2 TELEM 2
 * @value 3 TELEM 3
 * @value 4 TELEM 4
 * @value 101 TELEM 1
 * @value 102 TELEM 2
 * @value 103 TELEM 3
 * @value 104 TELEM 4
 * @reboot_required true
 * @group Sensors
 */
PARAM_DEFINE_INT32(LOADCELL_PORT, 3);

/**
 * Load cell baud rate
 *
 * @value 9600 9600
 * @value 19200 19200
 * @value 38400 38400
 * @value 57600 57600
 * @value 115200 115200
 * @reboot_required true
 * @group Sensors
 */
PARAM_DEFINE_INT32(LOADCELL_BAUD, 115200);

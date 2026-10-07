# DTRG MAVLink dialect and `DTRG_OFFBOARD`

The fork carries its own MAVLink dialect, `dtrg`, with one custom message,
`DTRG_OFFBOARD`, meant for 6-DOF offboard setpoints from a companion computer.
**Status: plumbing only.** PX4 receives the message and publishes and logs
it as the uORB topic `dtrg_custom`, but no controller reads that topic yet.
Today the offboard tilt for horizontal thrust goes through
`DEBUG_FLOAT_ARRAY` instead (see
[HT mode, Offboard](DTRG_Horizontal_Thrust_Mode.md#43-offboard-commanding-the-tilt)).

| | |
| --- | --- |
| Dialect | `src/modules/mavlink/mavlink/message_definitions/v1.0/dtrg.xml` (submodule `UoA-DTRG/dtrg-mavlink`, branch `dtrg-v1.17`); includes `development.xml` |
| Built with it | `px4_sitl_default`, `goosetech_fmu-v6xrt_default` (`CONFIG_MAVLINK_DIALECT="dtrg"`). The other boards use the upstream dialect and ignore the message. |
| Receiver | `MavlinkReceiver::handle_message_dtrg_offboard()` in `src/modules/mavlink/mavlink_receiver.cpp` |
| uORB | `dtrg_custom` (`msg/DtrgCustom.msg`), logged |
| Example | `src/examples/dtrg_test` (built on `px4_fmu-v6c_default`) |

---

## 1. The message

```xml
<message id="9003" name="DTRG_OFFBOARD">
  <description>Custom DTRG testing messages</description>
  <field type="float[6]" name="offboard_sp">Setpoint vector</field>
</message>
```

| Field | Type | Meaning (intended, from `DtrgCustom.msg`) |
| --- | --- | --- |
| `offboard_sp[0..2]` | float | x, y, z |
| `offboard_sp[3..5]` | float | pitch, roll, yaw |

Units and frame are not fixed by any consumer yet; agree them with whoever
writes the controller that reads `dtrg_custom`.

On reception PX4 copies the six values unchanged into `dtrg_custom` with the
receive time as timestamp. The message has no target system/component
fields, so any sender on any link is accepted.

The file `src/modules/mavlink/streams/DTRG_OFFBOARD.hpp` (sending the message
back out) is not registered as a stream and does not compile as it is; PX4
does not send `DTRG_OFFBOARD`.

---

## 2. How to use it

### Send it from a companion computer (pymavlink)

pymavlink does not ship the dialect, so generate it once from the XML:

```sh
python3 -m pymavlink.tools.mavgen --lang=Python --wire-protocol=2.0 \
    --output=dtrg.py src/modules/mavlink/mavlink/message_definitions/v1.0/dtrg.xml
```

```python
import time
import dtrg
from pymavlink import mavutil

m = mavutil.mavlink_connection("udpout:127.0.0.1:14580")  # SITL onboard port
m.mav = dtrg.MAVLink(m, srcSystem=1, srcComponent=191)
while True:
    m.mav.dtrg_offboard_send([1.0, 0.0, -2.0, 0.0, 0.0, 0.0])
    time.sleep(0.02)
```

Check on the PX4 console:

```sh
listener dtrg_custom
```

### Read it in a module

```cpp
#include <uORB/topics/dtrg_custom.h>

uORB::Subscription _dtrg_custom_sub{ORB_ID(dtrg_custom)};

dtrg_custom_s sp;
if (_dtrg_custom_sub.update(&sp)) {
	// sp.offboard_sp[0..5]
}
```

Check that the setpoint is recent (`sp.timestamp`) before using it: nothing
times it out.

### The `dtrg_test` example

`dtrg_test` is a minimal "hello sky" module. It prints the accelerometer at
5 Hz and publishes accelerometer X into `dtrg_custom.offboard_sp[1]`, as a
template for publishing the topic. Enable it with
`CONFIG_EXAMPLES_DTRG_TEST=y` and run `dtrg_test` on the console (it does not
return).

---

## 3. Adding messages to the dialect

1. Add the message to `dtrg.xml` in the `UoA-DTRG/dtrg-mavlink` repository
   (branch `dtrg-v1.17`), with an id that does not clash with
   `development.xml`/`common.xml`.
2. Update the submodule pointer in this repository.
3. Handle it in `MavlinkReceiver::handle_message()` inside
   `#if defined(MAVLINK_MSG_ID_<NAME>)`, so that boards built with another
   dialect still compile.
4. Build a board with `CONFIG_MAVLINK_DIALECT="dtrg"`.

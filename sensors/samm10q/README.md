# SAM-M10Q UART GPS Driver

This folder contains a UART driver for the SAM-M10Q GPS. It talks to the module over `Serial1` (UART) using u-blox's binary UBX protocol.

## How it's set up :D

| File | Purpose |
| --- | --- |
| `gps.h` / `gps.cpp` | The public API. |
| `ubx_messages.h` / `ubx_messages.cpp` | Builds the messages we send (config, polls) and decodes the ones we receive (NAV-PVT, ACK/NAK). |
| `ubx_parser.h` / `ubx_parser.cpp` | A state machine that takes bytes from the serial port one at a time and returns complete frames |
| `ubx_frame.h` / `ubx_frame.cpp` | The UBX frame struct |
| `test_gps.cpp` | A test program that prints position over USB serial. |

Every UBX frame looks like this:

```
B5 62 | class | id | length (2 bytes) | payload | checksum (2 bytes)
```

All multi-byte values are little-endian.

## Usage

Initialize the module once in `setup()` using `gps_init()`. It configures the module to:
- output UBX only.
- send a NAV-PVT (position/velocity/time) message at 5 Hz
- use the airborne (<4g) model

It returns `false` if the module doesn't respond or rejects the config. 

Call `gps_update()` periodically. It reads any waiting serial bytes and saves the latest fix.

```cpp
#include "gps.h"

void setup() {
    if (!gps_init()) {
        // module didn't respond
    }
}

void loop() {
    gps_update();

    if (gps_has_fix()) {
        const NavPvt* p = gps_latest();
    }
}
```

Other functions:
- `gps_has_fix()` returns true only if the latest solution is a valid 3D fix and is less than 1 second old.
- `gps_latest()` returns the last good fix. Check `gps_has_fix()` first, because the fix it returns may be old.
- `gps_fix_age_ms()` returns how long ago that fix was received. It returns `UINT32_MAX` if there has never been a fix.
- `gps_delta(origin, now)` returns the north, east and up offset of `now` from `origin` in meters. 




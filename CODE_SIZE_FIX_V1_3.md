# V1.3 - Arm Compiler 6.24 No-LTO fix

This version is based directly on V1.2.

The user's Arm Compiler for Embedded reports:

    error: use of LTO is disallowed in this variant of Arm Compiler for Embedded

Therefore V1.3 changes ONLY the unsupported LTO setting:

    v6Lto: 1 -> 0

The size-oriented AC6 optimization remains enabled:

    Optim = 7   (-Oz)

The following remain unchanged:
- all integrated application modules
- 6 CAN motors
- FDCAN2 500 kbit/s
- PB12 RX / PB13 TX
- HWT101
- K230
- QR scanner
- HMI / obstacle map / Dijkstra navigation
- PB4 gripper servo
- PE3 three-position carousel servo
- chassis / edge motion
- UART5 reserved interface

Also retained:
- One ELF Section per Function = enabled
- Optimize for Time = disabled
- no printf/sprintf/snprintf/freopen semihosting calls

Build:
1. Open MDK-ARM/27_carrycar.uvprojx
2. Project -> Rebuild all target files
3. Report either:
   - Program Size: Code=... RO-data=... RW-data=... ZI-data=...
   or
   - L6050U: code size = ... bytes

Do not regenerate with CubeMX before this rebuild.

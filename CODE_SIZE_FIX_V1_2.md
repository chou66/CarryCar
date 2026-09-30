# CarryCar G491 V1.2 - 32KB linker size fix

This project is based directly on:
`CarryCar_G491_Full_Integrated_V1_1_DriversFixed`

## Why V1.1 exceeded the linker limit

V1.1 used Arm Compiler 6.24 with the target optimization stored as:

    <Optim>4</Optim>

For AC6 this is the speed-oriented -O3 setting. It can increase image size
because of aggressive inlining and loop transformations.

## V1.2 changes

The functional source code and hardware mapping were NOT removed or reverted.

Keil target changes:
- Optimization: -Oz / image-size optimization (`<Optim>7</Optim>`)
- Link-Time Optimization: enabled (`<v6Lto>1</v6Lto>`)
- Optimize for Time: remains disabled (`<oTime>0</oTime>`)
- One ELF Section per Function: remains enabled (`<OneElfS>1</OneElfS>`)
- MicroLIB remains disabled to avoid changing C-library behavior during debugging

All full-integration modules remain in the target:
- 6 EMM CAN motors
- mecanum chassis / position feedback / edge motion
- HWT101
- QR scanner
- K230
- HMI touch screen
- obstacle map + Dijkstra navigation
- PB4 gripper servo
- PE3 three-position carousel servo
- UART5 reserved interface

CAN baseline remains:
- STM32G491VET6
- HSE 8 MHz -> SYSCLK 170 MHz
- FDCAN2 PB12 RX / PB13 TX
- Prescaler 17, TS1 15, TS2 4, SJW 4
- 500000 bit/s
- `MX_FDCAN2_Init()` followed by `fdcan2_UserInit()`

## Build

Open only:

    MDK-ARM/27_carrycar.uvprojx

Then:

    Project -> Rebuild all target files

Do not regenerate with CubeMX before this rebuild, because CubeMX can overwrite
project/source settings that were intentionally integrated.

If L6050U still appears, report the new byte count and the Program Size line.

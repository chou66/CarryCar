# UART5 + Wireless DAPLink + VOFA Debug

Open:
`MDK-ARM/27_carrycar.uvprojx`

This is a bench-debug build based on the V1.3 project that already passed linking.

## Wiring

- DAPLink RX <- PC12 / UART5_TX
- DAPLink TX -> PD2 / UART5_RX
- DAPLink GND -> GND
- UART5 = 115200, 8N1

SWD can remain connected at the same time.

This build initializes HWT101, UART5, FDCAN2 and TIM3 servos. It intentionally does not run the full navigation/chassis state machine, so no motor moves automatically.

## MCU -> VOFA

Use the DAPLink virtual COM port and select JustFloat.

10 channels are sent every 40 ms:

1. roll
2. pitch
3. relative yaw
4. IMU online (0/1)
5. can_error_step
6. can_error_count
7. can_busoff_cnt
8. can_rx_flag
9. last command
10. last result

last result: 0 none, 1 accepted, 2 bad parameter, 3 unknown command.

## VOFA -> MCU

Use HEX send mode. Every frame is exactly:

`AA 55 CMD P1 P2 P3 P4`

Examples:

- Stop all 6 motors: `AA 55 01 00 00 00 00`
- Zero yaw: `AA 55 02 00 00 00 00`
- Enable ID1: `AA 55 10 01 01 00 00`
- Disable ID1: `AA 55 10 01 00 00 00`
- ID1 DIR=0 200RPM ACC=20: `AA 55 11 01 00 14 14`
- ID1 DIR=1 200RPM ACC=20: `AA 55 11 01 01 14 14`
- Stop ID1: `AA 55 12 01 00 00 00`

For IDs 2..6, replace the ID byte.

Gripper PB4:
- open: `AA 55 20 00 00 00 00`
- mid: `AA 55 20 01 00 00 00`
- close: `AA 55 20 02 00 00 00`

Carousel PE3:
- slot0: `AA 55 21 00 00 00 00`
- slot1: `AA 55 21 01 00 00 00`
- slot2: `AA 55 21 02 00 00 00`

Raw servo pulse, 1500us = 0x05DC:
- gripper: `AA 55 22 00 05 DC 00`
- carousel: `AA 55 22 01 05 DC 00`

Motor velocity uses P3 in units of 10 RPM. This debug build clamps speed to 500 RPM.

## First test

Raise all wheels off the ground, then:

1. Rebuild and flash.
2. Open VOFA at 115200.
3. Confirm JustFloat channels update.
4. Send `AA 55 10 01 01 00 00`.
5. Send `AA 55 11 01 00 14 14`.
6. Verify only ID1 turns.
7. Send `AA 55 12 01 00 00 00`.

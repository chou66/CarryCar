These files are copied from the user's first, already-tested STM32F4 project
for traceability only. They are NOT compiled in the STM32G491 target.

Integrated/adapted equivalents:
- HWT101 parser -> Hardware/hwt101_uart.c + hwt101_compat.c, USART1 PC4/PC5
- K230 parser -> Hardware/k230.c, UART4 PC10/PC11
- MJ6000 scanner idea -> Hardware/qr_scanner.c, USART2 PA2/PA3
- servo behavior -> Hardware/servo.c, TIM3 PB4/PE3
- competition_task.c is retained only as a reference because the current
  obstacle/HMI/navigation workflow and 6-motor mechanism are structurally
  different; blindly compiling the old task would create wrong hardware actions.

# XMC4400 Motor Control Kit

### Overview
This kit includes XMC4400F motor control board connected to a standard motor drive power board.

<img src=".\Images\Kit.jpg" alt="Kit" width="1000"/>

- **A**: XMC4400 motor control board
- **B**: Inverter board
- **C**: USB connector for on-board debugger and UART interface
- **D**: Command potentiometer
- **E**: Reset push button
- **F**: Direction indicator LED
- **G**: Fault indicator LED (both hardware and software faults)
- **H**: Hall connector
- **I**: Encoder connector
- **J**: XMC4400 microcontroller chip
- **K**: Individual phase shunts used in three shunt configuration
- **L**: DC-link shunt used in single shunt configuration
- **M**: Motor connector
- **N**: DC power connector

The inverter board is rated at 250W output power and 24V input DC voltage.

### Features
Key features of XMC4400F motor control board (XMC4400-F100K512):

- ARM-Cortex M4 120MHz with Floating Point Unit
- 512kB flash memory
- 80kB SRAM
- Supporting Segger J-Trace/J-Link
- Supporting Infineon Miniprog

Key features of the standard motor drive power board:

- Integrated on-board power supplies
- 3x highly accurate current sense amplifiers
- Various fault detections handled by HW


### Errata

Control board:
- Change R343 to 0&Omega;
- Change R344 to 0&Omega;
- Change R345 to 0&Omega;
- Change R349 to 19.6k&Omega;
- Change R350 to 1.3k&Omega;
- Change R351 to 200k&Omega;
- Connect a wire jumper between X302-B8 and X202-2 on the back of the board

Inverter board:
- No errata
# Internal release v1.1.0
This release is for internal evaluation by Infineon employees only.
Please do not share this firmware with customer as there are pending patents to be filed and go-to-market strategy has not been finalized.
For feedbacks and questions please contact hamid.behjati@infineon.com.
### Key features of Andromeda application firmware:

* Supporting two prominent control methods for SPMs and IPMs, namely,
  * Rotor Frame Orientation (RFO)
  * Stator Frame Orientation (SFO)

* Advnaced motor control capabilities, namely,
  * Speed control (RFO/SFO)
  * Current control (RFO)
  * Torque control (SFO)
  * Votlage control (RFO/SFO)

* Adaptive sensorless observer with minimal phase distortion capable of observing
  * Rotor angle (RFO/SFO)
  * Speed (RFO/SFO)
  * Stator angle (SFO)
  * Stator flux magnitude (SFO)
  * Load angle (SFO)

* Floating point implemenation of all control features
* Fault detection, collection, reaction, and clearance for various HW and FW faults
* Supporting various sensorless startup algorithms, namely
  * Open-loop volt/hz
  * Rotor pre-alignment
  * Six pulse injection
  * High freqeuncy injection							    
* Auxiliary motor control features, including
  * Maximum Torque Per Amp (MTPA)
  * Maximum Torque Per Volt (MTPV)
  * Field weakening
  * Active damping
  * Motor I2T protection
  * Various voltage modulation methods
  * Motor control and DSP math library
  * RTOS support for implementation of secondary controllers
* Regression testing framework capable of running on PC
* Software In the Loop (SIL) simulation environment running in MATLAB/Simulink

### Key features of Andromeda evaluation kit:
This kit includes PSOC6 motor control board connected to 6EDL7141 inverter and gate driver board.
* Key features of PSOC6 motor control board (CY8C6244AZI-S4D93)
  * 150MHz ARM-Cortex M4 with Floating Point Unit
  * 100MHz ARM-Cortex M0+
  * 256KB flash memory
  * 128KB SRAM
  * Supporting Segger J-Trace/J-Link
  * Supporting Infineon Miniprog
* Key features of 6EDL7141 inverter and gate driver board
  * Adjustable gate drive parameters to optimize slew rate and EMI
  * Integrated on-board power supplies
  * 3x highly accurate current sense amplifiers
  * Various fault detections handled by HW


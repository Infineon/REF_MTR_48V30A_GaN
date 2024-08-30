# Internal release v1.3.0
This release is for internal evaluation by Infineon employees only.
Please do not share this firmware with customers as there are pending patents to be filed and go-to-market strategy has not been finalized.
For feedbacks and questions please contact hamid.behjati@infineon.com.

### Key features included in this release:

* Adding support for **saving the tuning parameters in flash**
  * When upgrading the firmware, the tuning parameters will be retained. Therefore, there is no need to retune after upgrading the firmware.
  * If there is a parameter upgrade, or first time programming the flash, the tuning parameters are initialized and saved in flash.

* Adding support for **measuring the phase voltages**
  * Phase voltages can be used in catch spin or dyno mode so that the observer can lock while the motor is spinning.
  * Phase voltages are sampled only when they are used by firmware and the ADC samplings are time-multiplexed to save execution time.

* Adding **dyno control modes** in both RFO (current control) and SFO (torque control).
  * Dyno modes allow for creating a back-to-back motor-generator system where one motor controls the speed and the other motor controls the torque or current.
  * Both motors can be in FOC sensorless mode. There is no need for position sensors to build a dyno system using Andromeda.

* Adding support for **handshaking between GUI and firmware**
  * This handshaking mechanism allows GUI to send requests to firmware to execute various functions.
  * Then firmware will acknowledge receiving the request, and start working on it.
  * After GUI's request is serviced, firmware signals to the GUI that the function execution is done.

* Adding **CPU-utilization measurement timers and methods** to measure the utilization of critical ISRs.

* Code optimizations have been done to **maximize the execution speed** and minimize the CPU utilization

* Adding support for **high-sampling-rate oscilloscope functionality** that can be used by GUI
  * Firmware samples at the maximum sampling-rate (up to 40kHz) and stores the results in a buffer, then transmits them to GUI.
  * Therefore, the communication channel's bandwidth has no effect on the maximum achievable sampling rate.

* Adding support for **identification by GUI**, i.e. kit ID, build config ID, chip ID, firmware version, and parameters version.

* An **automation suite** is created to automate the build and regression testing process and
  * Ensure there are no build errors or warnings in IAR, GCC, or Visual Studio (using RFO, SFO, or TBC)
  * Run the regression tests on PC (using RFO, SFO, or TBC)

* All following **control modes** are supported:

| Control Type | Controlled Entity | Feedback Type | Startup Method |
| ------------ | ------------ | ------------ | ------------ |
| Open-Loop | Voltage (Volt/Hz) | N.A. | N.A. |
| FOC in RFO | Current | Sensorless | Rotor Pre-Alignment |
| FOC in RFO | Current | Sensorless | Six Pulse Injection |
| FOC in RFO | Current | Sensorless | High Frequency Injection |
| FOC in RFO | Current | Sensorless | Dyno Mode |
| FOC in RFO | Current | Hall Sensor | N.A. |
| Block Commutation (TBC) | Current | Hall Sensor | N.A. |
| FOC in SFO | Torque | Sensorless | Rotor Pre-Alignment |
| FOC in SFO | Torque | Sensorless | Six Pulse Injection |
| FOC in SFO | Torque | Sensorless | High Frequency Injection |
| FOC in SFO | Torque | Sensorless | Dyno Mode |
| FOC in RFO or SFO | Speed | Sensorless | Rotor Pre-Alignment |
| FOC in RFO or SFO | Speed | Sensorless | Six Pulse Injection |
| FOC in RFO or SFO | Speed | Sensorless | High Frequency Injection |
| FOC in RFO or SFO | Speed | Sensorless | Open-Loop Volt/Hz |
| FOC in RFO | Speed | Hall Sensor | N.A. |
| Block Commutation (TBC) | Speed | Hall Sensor | N.A. |

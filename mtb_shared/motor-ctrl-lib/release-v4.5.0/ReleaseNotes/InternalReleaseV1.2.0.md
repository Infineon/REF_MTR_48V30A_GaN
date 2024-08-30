# Internal release v1.2.0
This release is for internal evaluation by Infineon employees only.
Please do not share this firmware with customer as there are pending patents to be filed and go-to-market strategy has not been finalized.
For feedbacks and questions please contact hamid.behjati@infineon.com.
### Key features included in this release:

* Adding support for **hall sensors**

* Adding support for **sensored field oriented control** using hall sensors

* Adding support for **trapezoidal block commutation** using hall sensors

* Adding support for **IAR Embedded Workbench**. Now Andromeda can be deployed in any of the following environments:
  * **ModusToolbox**: GCC compiler; PSOC6 target; For application deployment.
  * **IAR**: IAR compiler; PSOC6 target; For application deployment.
  * **Visual Studio**: Visual Studio compiler; x86 target; For unit and regression testing on PC.
  * **MATLAB/Simulink**: MinGW compiler; x86/x64 target; For Software In the Loop (SIL) testing on PC.

* All following **control modes** are supported:

| Control Type | Controlled Entity | Feedback Type | Startup Method |
| ------------ | ------------ | ------------ | ------------ |
| Open-Loop | Voltage (Volt/Hz) | N.A. | N.A. |
| FOC in RFO | Current | Sensorless | Rotor Pre-Alignment |
| FOC in RFO | Current | Sensorless | Six Pulse Injection |
| FOC in RFO | Current | Sensorless | High Frequency Injection |
| FOC in RFO | Current | Hall Sensor | N.A. |
| Block Commutation (TBC) | Current | Hall Sensor | N.A. |
| FOC in SFO | Torque | Sensorless | Rotor Pre-Alignment |
| FOC in SFO | Torque | Sensorless | Six Pulse Injection |
| FOC in SFO | Torque | Sensorless | High Frequency Injection |
| FOC in RFO or SFO | Speed | Sensorless | Rotor Pre-Alignment |
| FOC in RFO or SFO | Speed | Sensorless | Six Pulse Injection |
| FOC in RFO or SFO | Speed | Sensorless | High Frequency Injection |
| FOC in RFO or SFO | Speed | Sensorless | Open-Loop Volt/Hz |
| FOC in RFO or SFO | Speed | Hall Sensor | N.A. |
| Block Commutation (TBC) | Speed | Hall Sensor | N.A. |

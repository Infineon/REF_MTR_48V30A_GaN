# Internal release v1.4.0

This release is for internal evaluation by Infineon employees only.
Please do not share this firmware with customers as there are pending patents to be filed and go-to-market strategy has not been finalized.
For feedbacks and questions please contact hamid.behjati@infineon.com.

### Key features included in this release:

* Changing Andromeda from an **application** to a **library** in ModusToolbox:
  
  * One can create multiple application code examples while cloning and keeping one copy of the library.
  * The application examples will be very thin (inlcuding only main.c). They only configure the library (by setting control mode etc.) and utilize it.
  * ModusToolbox library manager can be used to upgrade or checkout different versions of the library.

* Adding support for **XMC7000** kit and **XMC4000** kit in addition to **PSOC6** kit:
  
  * Each kit comes with a separate Board Support Package (BSP) that can be selected when creating a project in ModusToolbox.
  * The library is hardware-agnostic and can support multiple device categories e.g. CAT1A (PSOC6), CAT1C (XMC7000), and CAT3 (XMC4000).

* Adding support for **single shunt** configuration in addition to **three shunts** configuration
  * All BSPs support both configurations.
  * There is no need to change the BSP when switching between these configurations.

* Adding **motor profiler** feature:
  
  * One can use motor profiler to extract the motor parameters such as q/d-axis inductances, stator resistance, and flux linkage.
  * This feature facilitates tuning of the system when new motors are used.

* Adding **bypassing current loop** capability in TBC:
  
  * This feature will support BLDC applications that have no current sensors.

* Adding **trapezoidal commutation** feature in TBC:
  
  * This is a new IP that controls the current waveforms to be trapezoidal with ramp-up and ramp-down phases at each revolution.
  * This method improves the efficiency compared to conventional block commutation method.

* All following **control modes** are supported:

| Control Type | Controlled Entity | Feedback Type | Startup Method |
|:------------:|:------------:|:------------:|:------------:|
| Open-Loop | Voltage | N.A. | N.A. |
| FOC in RFO | Current | Sensorless | Rotor Pre-Alignment |
| FOC in RFO | Current | Sensorless | Six Pulse Injection  |
| FOC in RFO | Current | Sensorless | High Frequency Injection |
| FOC in RFO | Current | Sensorless | Dyno Mode |
| FOC in RFO | Current | Hall Sensor | N.A. |
| TBC in BC | Current | Hall Sensor | N.A. |
| TBC in TC | Current | Hall Sensor | N.A. |
| FOC in SFO | Torque | Sensorless | Rotor Pre-Alignment |
| FOC in SFO | Torque | Sensorless | Six Pulse Injection |
| FOC in SFO | Torque | Sensorless | High Frequency Injection |
| FOC in SFO | Torque | Sensorless | Dyno Mode |
| FOC in RFO or SFO | Speed | Sensorless | Rotor Pre-Alignment |
| FOC in RFO or SFO | Speed | Sensorless | Six Pulse Injection |
| FOC in RFO or SFO | Speed | Sensorless | High Frequency Injection |
| FOC in RFO or SFO | Speed | Sensorless | Open-Loop Volt/Hz |
| FOC in RFO | Speed | Hall Sensor | N.A. |
| TBC in BC | Speed | Hall Sensor | N.A. |
| TBC in TC | Speed | Hall Sensor | N.A. |

| Acronym | Expansion |
|:------------:|:------------|
| **FOC** | Field Oriented Control |
| **RFO** | Rotor Frame Orientation |
| **SFO** | Stator Frame Orientation |
| **TBC** | Trapezoidal or Block Commutation |
| **TC** | Trapezoidal Commutation |
| **BC** | Block Commutation |
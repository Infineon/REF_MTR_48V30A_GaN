# Internal release v1.5.0

This is the last internal release before public release of all assets.
This release is based on the original Bitbucket repositories but after public release, Github/Gitlab will be the target repositories.

### Key features included in this release:

* Adding support for **PSOC-C3** kit and in addition to **PSOC6** kit, **XMC7000** kit, and **XMC4000** kit:
  
  * Each kit comes with a separate Board Support Package (BSP) that can be selected when creating a project in ModusToolbox.
  * The library is hardware-agnostic and can support multiple device categories e.g. CAT1A (PSOC6), CAT1B (PSOC-C3), CAT1C (XMC7000), and CAT3 (XMC4000).

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
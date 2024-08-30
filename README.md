# REF_MTR_48V30A_GaN Firmware Development Environment

<a href="https://www.infineon.com">
<img src="./motor_ctrl_app/Images/Logo.svg" align="right" alt="Infineon logo">
</a>

<p>Infineon GitHub repository for REF_MTR_48V30A_GaN.</p>

## Instruction 
- Install ModusToolbox V3.2
- "Import Existing Application In-Place" and open the project "motor_ctrl_app"
- Install Micrium µC/Probe™ XMC™ software, open the workspace file "REF_MTR_48V30A_GaN_GUI.wspx" with µC/Probe™ to use the GUI.

# REF_MTR_48V30A_GaN Firmware

The firmware of REF_MTR_48V30A_GaN is based on Infineon Motor Control Library (IMCL), with specific modifications on BSP and periphral resources' adoption due to MCU and hardware differences. 

IMCL is a versatile motor control library that can be seemlessly deployed to multiple hardware and software platforms.

The pillars of IMCL ecosystem are the following:

- **Scalability**
    - Supporting multiple <span style="color:blue">Hardware</span> platforms
    - Supporting multiple <span style="color:blue">Software</span> platforms
    - Supporting multiple <span style="color:blue">Control</span> methodologies
    - Supporting multiple <span style="color:blue">Applications</span>
- **Maintainability**
    - Including a comprehensive <span style="color:blue">Regression Test</span> framework
    - Including an <span style="color:blue">Automated Build and Release Check</span> suite
- **Advanced Features**
    - Supporting <span style="color:blue">Rotor Field Oriented (RFO)</span> control
    - Supporting <span style="color:blue">Stator Field Oriented (SFO)</span> control
    - Supporting <span style="color:blue">Trapezoidal or Block Commutation (TBC)</span> control
- **Floating Point Base**
- **GUI Integration**
    - <span style="color:blue">Configurator</span> view
    - <span style="color:blue">Test Bench</span> view
    - <span style="color:blue">Tools</span> such as Oscilloscope, MOSFET tuner, GUI builder, etc.
- **Model Based Simulation**
    - Also known as <span style="color:blue">Software In the Loop</span>

Almost all the pillars mentioned above are still supported on REF_MTR_48V30A_GaN after firmware adaptions on IMCL, **except the "GUI Integration"**. 

A simpler GUI workspace file named "REF_MTR_48V30A_GaN_GUI.wspx" is provided together with this firmware. This workspace can be open with uC-Probe and works as the GUI to assist the motor tuning.

## REF_MTR_48V30A_GaN Hardware

The important components of this reference board for motor control applications are listed below:

<img src=".\motor_ctrl_app\Images\REF_MTR_48V30A_GaN-top.png" alt="REF_MTR_48V30A_GaN-top" width="1000"/>
<img src=".\motor_ctrl_app\Images\REF_MTR_48V30A_GaN-bottom.png" alt="REF_MTR_48V30A_GaN-bottom" width="1000"/>

- **A**: DC power input
- **B**: Motor 3 phases output
- **C**: Current sensors
- **D**: Encoder / CAN / UART connection
- **E**: XMC-Link debug connector
- **F**: CAN IC
- **G**: XMC4200-Q48x256(VQFN-48)
- **H**: LDO (5V->3.3V)
- **I**: Buck (DC input -> 5V)
- **J**: Fuse
- **K**: 5V (power up indication)
- **L**: GaN (IQC033G10LS1SC)
- **M**: Gate driver (1EDN71x6)
- **N**: Temperature sensor (MCP9700AT-E/LT or MAX6612MXK+T)

## Control Methodologies

There are three major <span style="color:blue">control types</span> that are supported, namely
- Rotor Field Oriented (RFO) control
- Rotor Field Oriented (RFO) control
- Trapezoidal or Block Commutation (TBC) control

These control types are selectable as <span style="color:blue">build configurations</span> in ModusToolbox, IAR, Visual Studio, and MATLAB.

By choosing a specific build configuration, only the code pertaining to that build configuration is compiled and included.
There are also common code blocks among all build configurations that are always included.

After selecting the control type through build configurations, the user can also choose the *controlled entity*, *feedback type*, and *startup method* by assigning the corresponding parameters either in the code before compilation or at runtime through GUI:

<img src=".\motor_ctrl_app\Images\Control-Methods.png" alt="Control-Methods" width="720"/>

There are 23 different permutations of control type, control entity, feedback type, and statup method that are supported as illustrated in the table above.
In addition, both three-shunt and single-shunt configurations are supported, which result in more flexibility in supporting various applications.
It should be noted that the user can either include or bypass the current loop when using *TBC in BC* mode.
Bypassing the current loop can address low-cost BLDC applications with no shunts or ADCs.

<b> Note: By default, the following mode is chosen in the REF_MTR_48V30A_GaN firmware. This can be changed according to application needs.

 FOC in RFO, Speed, Sensorless, Rotor pre-Alignment</b>

**release-v1.5.0**
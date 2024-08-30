%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
% an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
%
% This software, including source code, documentation and related
% materials ("Software") is owned by Cypress Semiconductor Corporation
% or one of its affiliates ("Cypress") and is protected by and subject to
% worldwide patent protection (United States and foreign),
% United States copyright laws and international treaty provisions.
% Therefore, you may use this Software only as provided in the license
% agreement accompanying the software package from which you
% obtained this Software ("EULA").
% If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
% non-transferable license to copy, modify, and compile the Software
% source code solely for use in connection with Cypress's
% integrated circuit products.  Any reproduction, modification, translation,
% compilation, or representation of this Software except as specified
% above is prohibited without the express written permission of Cypress.
%
% Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
% EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
% reserves the right to make changes to the Software without notice. Cypress
% does not assume any liability arising out of the application or use of the
% Software or any product or circuit described in the Software. Cypress does
% not authorize its products for use in any products where a malfunction or
% failure of the Cypress product may reasonably be expected to result in
% significant property damage, injury or death ("High Risk Product"). By
% including Cypress's product in a High Risk Product, the manufacturer
% of such system or application assumes all risk of such use and in doing
% so agrees to indemnify Cypress against all liability.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear variables;
close all;
clc;

%% NTC Parameters
% v (normalized voltage) = (Rl||Rntc) / (Rp + Rl||Rntc) * Vcc_pow / Vcc_ctrl
% where Vcc_pow = 5.0V and Vcc_ctrl = 5.0V or 3.3V
% Rntc = R0 * exp(B * (1/T - 1/T0))
R0 = 100;       % [kOhm]
T0 = 25 + 273;  % [K]
B = 4300;       % [K]
Rp = 100;       % [kOhm] - pull up resistor
Rl = 196;       % [kOhm] - pull down resistor - only for xmc4400 5V-to-3.3V level shifter

N = 16;         % [#] LUT width
N0 = 2048;      % [#] Width for accuracy testing
k = 3.3/5.0;    % [%] Control Vcc to Power Vcc ratio - only for xmc4400

%% LUT Initialization - 250W Power Board + XMC7200 Control Board
v = [1/(N+1):1/(N+1):1-1/(N+1)];    % Normalized with respect to Vcc = 5V
R = Rp*v./(1-v);
T = 1./(1/T0+1/B*log(R/R0)); C = T-273;
figure, plot(v,C,'b'), xlabel('Normalized voltage [%]'), ylabel('T_{NTC} [C]'), title('XMC7200'), hold on;
disp('XMC7200')
v
C
step = 1/(N+1)
step_inv = 1/step

v = [1/(N+1):1/(N0+1):1-1/(N+1)];    % Normalized with respect to Vcc = 5V
R = Rp*v./(1-v);
T = 1./(1/T0+1/B*log(R/R0)); C = T-273;
plot(v,C,'r'), xlabel('Normalized voltage [%]'), ylabel('T_{NTC} [C]'), title('XMC7200');
grid minor;

%% LUT Initialization - 250W Power Board + XMC4400 Control Board
v = [1/(N+1):1/(N+1):1-1/(N+1)];    % Normalized with respect to Vcc = 3.3V

Req = Rp*(k*v)./(1-(k*v));
R = 1./(1./Req - 1/Rl);
T = 1./(1/T0+1/B*log(R/R0)); C = T-273;
figure, plot(v,C,'b'), xlabel('Normalized voltage [%]'), ylabel('T_{NTC} [C]'), title('XMC4400'), hold on;
disp('XMC4400')
v
C
step = 1/(N+1)
step_inv = 1/step

v = [1/(N+1):1/(N0+1):1-1/(N+1)];    % Normalized with respect to Vcc = 3.3V
Req = Rp*(k*v)./(1-(k*v));
R = 1./(1./Req - 1/Rl);
T = 1./(1/T0+1/B*log(R/R0)); C = T-273;
plot(v,C,'r'), xlabel('Normalized voltage [%]'), ylabel('T_{NTC} [C]'), title('XMC4400');
grid minor;
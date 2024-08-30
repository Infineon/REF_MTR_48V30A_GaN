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

% Description: Parameter Definition File for Motor Control SIL

%% MATLAB Overwrite Flag
MATLAB_Overwrite_Params = true;

%% Initializing MATLAB-Specific Parameters (Only Used in MATLAB):
m = struct;

% System Parameters
m.sys.tsim = 6.0;                               % [sec]

% Command Parameters
m.digital_in = uint32(1);                       % []
m.pot.init = 0.0;                               % [%]
m.pot.final = 1.0;                              % [%]
m.pot.t_start = 0.5;                            % [sec]
m.pot.t_slope = 2.5;                            % [sec]
m.pot.t_stop =  3.5;                            % [sec]
m.pot.slope_lim = abs((m.pot.final-m.pot.init)/m.pot.t_slope);    % [%/sec]

%% Overwriting C-Specific Parameter Structure (Used in Both C-Code and MATLAB):
if (MATLAB_Overwrite_Params==false)
    return;
else
    fprintf('\n%s\n','Overwriting FW Parameteres by MATLAB...');
end
                   
% Motor Parameters
if strcmp(ctrl_method,'SFO')
    n_1d_table = length(p.motor.mtpa_lut.y);                      % [#]
    % MTPA table
    m.motor.mtpa_lut.T = linspace(0,p.motor.T_max,n_1d_table);    % [Nm]
    m.motor.mtpa_lut.la_d = zeros(1,n_1d_table);                  % [Wb]
    m.motor.mtpa_lut.la_q = zeros(1,n_1d_table);                  % [Wb]
    m.motor.mtpa_lut.la = zeros(1,n_1d_table);                    % [Wb]
    la_d_poly = (3*p.motor.P/4)^2*p.motor.zeta^3/(1-p.motor.zeta)/p.motor.ld^2*conv([1 -p.motor.lam],conv(conv([-(p.motor.zeta-1)/p.motor.zeta p.motor.lam],[-(p.motor.zeta-1)/p.motor.zeta p.motor.lam]),[-(p.motor.zeta-1)/p.motor.zeta p.motor.lam]));   % []
    for i=1:n_1d_table
        root = roots(la_d_poly+[0 0 0 0 -(m.motor.mtpa_lut.T(i))^2]);
        m.motor.mtpa_lut.la_d(i) = root(4);
        m.motor.mtpa_lut.la_q(i) = sqrt(abs(p.motor.zeta^3/(1-p.motor.zeta)*(m.motor.mtpa_lut.la_d(i)-p.motor.lam)*(p.motor.lam-(p.motor.zeta-1)/p.motor.zeta*m.motor.mtpa_lut.la_d(i))));
        m.motor.mtpa_lut.la(i) = sqrt(abs(m.motor.mtpa_lut.la_d(i)^2+m.motor.mtpa_lut.la_q(i)^2));
    end
    m.test.mtpa_lut.T = m.motor.mtpa_lut.T;                       % [Nm]
    m.test.mtpa_lut.la_d = zeros(1,n_1d_table);                   % [Wb]
    m.test.mtpa_lut.la_q = zeros(1,n_1d_table);                   % [Wb]
    m.test.mtpa_lut.la = zeros(1,n_1d_table);                     % [Wb]
    m.test.mtpa_lut.la_d(1) = p.motor.lam;                        % [Wb]
    m.test.mtpa_lut.la_q(1) = 0;                                  % [Wb]
    m.test.mtpa_lut.la(1) = p.motor.lam;                          % [Wb]
    c1 = 0.5*(p.motor.ld/(p.motor.zeta-1)/0.75/p.motor.P*p.motor.mtpa_lut.x_step)^2;    % [#]
    c2 = p.motor.zeta/(p.motor.zeta-1)*p.motor.lam;               % [#]
    c3 = (p.motor.zeta-0.75)/(p.motor.zeta-1)*p.motor.lam;        % [#]
    c4 = p.motor.zeta^2;                                          % [#]
    for i=1:(n_1d_table-1)
        m.test.mtpa_lut.la_d(i+1) = m.test.mtpa_lut.la_d(i)+c1/(m.test.mtpa_lut.la_d(i)-c2)^2/(m.test.mtpa_lut.la_d(i)-c3)*(i-0.5);
        m.test.mtpa_lut.la_q(i+1) = sqrt(c4*(m.test.mtpa_lut.la_d(i+1)-p.motor.lam)*(m.test.mtpa_lut.la_d(i+1)-c2));
        m.test.mtpa_lut.la(i+1) = sqrt(m.test.mtpa_lut.la_d(i+1)^2+m.test.mtpa_lut.la_q(i+1)^2);
    end
    m.motor.la_max=max(p.motor.mtpa_lut.y);
    clear la_d_poly i root c1 c2 c3 c4 i;
    % MTPV table
    m.test.mtpv_lut.la = linspace(0,m.motor.la_max,n_1d_table);   % [Wb]
    m.test.mtpv_lut.delta_max = zeros(1,n_1d_table);              % [Ra]
    m.test.mtpv_lut.Tmax = zeros(1,n_1d_table);                   % [Nm]
    m.test.mtpv_lut.delta_max(1) = pi/2;                          % [Nm]
    m.test.mtpv_lut.Tmax(1) = 0;                                  % [Nm]
    c1 = (0.75*p.motor.P)*p.motor.mtpv_lut.x_step^2/p.motor.ld*(p.motor.zeta-1)/p.motor.zeta*p.motor.mtpv_margin;
    c2 = 0.25*p.motor.lam/p.motor.mtpv_lut.x_step*p.motor.zeta/(p.motor.zeta-1);
    for i=2:n_1d_table
        j=i-1;
        c3 = c2/j;
        c_cos = c3-sqrt(c3^2+0.5);
        c_sin = sqrt(1-c_cos^2);
        m.test.mtpv_lut.delta_max(i)=atan2(c_sin,c_cos);
        m.test.mtpv_lut.Tmax(i)=c1*j^2*c_sin*(4*c3-c_cos);
    end
    clear k delta_max c1 c2 c3 c_sin c_cos i j n_1d_table;
end

%% Creating Single and Double Precision Versions of Parameters
p_single = ConvertToSinglePrecision(p);
p = ConvertToDoublePrecision(p);
m = ConvertToDoublePrecision(m);

%% Checking Structural Equality (Not a Numerical Check)
if (~isStructEqual(p_single,p_default))
    error('Structure of the C-code parameters changed by MATLAB!');
end

%% Creating Simulink Parameters for S-Functions
pp=Simulink.Parameter(p_single);
pp.DataType='Bus: PARAMS_t';

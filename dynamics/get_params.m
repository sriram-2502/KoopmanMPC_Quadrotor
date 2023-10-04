function param = get_params(nosie_flag)
% Output
% parameters for the quadrotor
% params.m          - mass of the quadrotor
% params.J          - iniertia of the quadrotor (in body frame)
% params.d          - distance between com and rotors (in body grame)
% params.c_torque   - torque constant

%% physical parameters

% reference - Aeiral robots course notes
% this is for a crazyflie
% m = 0.18; % kg
% J = [0.00025,   0,          2.55e-6;
%      0,         0.000232,   0;
%      2.55e-6,   0,          0.0003738];

% reference - Geometric Tracking Control of a Quadrotor UAV on SE(3)
% Quadrotor
J1 = 0.02;
J2 = 0.02;
J3 = 0.04;
param.J = diag([J1, J2, J3]);

param.m = 2;

param.d = 0.169;
param.ctf = 0.0135;

% Fixed disturbance
if(nosie_flag)
%     param.x_delta = [0.5, 0.8, -1]';
%     param.R_delta = [0.2, 1.0, -0.1]';
    param.x_delta = 1*[0.5, 0.8, -1]';
    param.R_delta = 1*[0.2, 1.0, -0.1]';
else
    param.x_delta = [0, 0, 0]';
    param.R_delta = [0, 0, 0]';
end

% Other parameters
param.g = 9.81;


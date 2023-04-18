function params = get_params()
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
m = 4.34; % kg
J = diag([0.0820, 0.0845, 0.1377]);

% db = 0.315; % distance between the com and rotors
% c_torque = 8.004e-4; % torque constrant

g = 9.81; % gravity

params.mass = m;
params.J = J;
%params.d = db;
%params.c_troque = c_torque;
params.g = g;


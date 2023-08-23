function [k, param]=get_control_gains(param)

%% Controller gains
k.x = 10;
k.v = 8;
k.i = 10;
param.c1 = 1.5;
param.sigma = 10;

% Attitude
k.R = 1.5;
k.W = 0.35;
k.I = 10;
param.c2 = 2;

% Yaw
k.y = 0.8;
k.wy = 0.15;
k.yI = 2;
param.c3 = 2;
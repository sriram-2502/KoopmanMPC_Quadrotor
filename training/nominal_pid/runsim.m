close all;
clear;

addpath('utils');

%% pre-calculated trajectories
% trajhandle = @traj_line;
%trajhandle = @traj_helix;

%% Trajectory generation with waypoints
%% You need to implement this
trajhandle = @traj_generator;
waypoints = [0    0   0;
             1    1   1;
             2    0   2;
             3    -1  1;
             4    0   0]';
trajhandle([],[],waypoints);


%% controller
controlhandle = @controller;


% Run simulation with given trajectory generator and controller
% state - n x 13, with each row having format [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]
% control - n x 4 with each row having format [f_next, M1, M2, M3]
[t, state, control] = simulation_3d(trajhandle, controlhandle);

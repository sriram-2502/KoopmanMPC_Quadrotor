function [X, U, X1, X2, U1, U2] = get_pid_trajectories(X0, n_traj,traj_type,height)
% function to random trajectories
% Inputs
% X0                : initial condition
% n_traj            : number of trajectories to simulate
% t_traj            : trajectory length for each control input 
% Outputs
% X                 : random trajectories generated with constant control for
% each trajectory
% size of X         : 18 * (n_control x len(t_traj))
% U                 : random control inputs for each trajectory
% size of U         : 4 * (n_control x len(t_traj))

addpath('nominal_pid/utils');
addpath('nominal_pid');

%% get robot params
params = get_params();


%% simulate a pid controller to follow waypoints
% states X = [x dx R wb]'
% initial condition is from rest (from ground)
X = []; X1=[]; X2=[];
U = []; U1=[]; U2=[];

for i=1:n_traj
    % get control
    controlhandle = @controller;

    trajhandle = @traj_generator;
    if traj_type == 'hover'
        waypoints = zeros(3);
        waypoints(:,end) = linspace(0,height(i),3);
    end
    trajhandle([],[],waypoints);

    % simulate ode
    % state - n x 13, with each row having format [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]
    % control - n x 4 with each row having format [f_next, M1, M2, M3]
    [t,x,u] = simulation_3d(trajhandle, controlhandle);
    x(:,7) = []; % remove qw

    % collect data
    X = [X,x']; % [X(t1), X(t2), ..., X(tn)] stacked for each control input n
    U = [U, u'];

    % seperate data into snapshots
    X1 = [X1,x(1:end-1,:)'];
    X2 = [X2,x(2:end,:)'];
    U1 = [U1,u(1:end-1,:)'];
    U2 = [U2,u(2:end,:)'];
end


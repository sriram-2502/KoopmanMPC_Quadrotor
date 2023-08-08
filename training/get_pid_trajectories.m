function [X, U, X1, X2, U1, U2] = get_pid_trajectories(traj_param)
% function to random trajectories
% Inputs

% parameters        : structure of required trajectory parameters
% Outputs
% X                 : random trajectories generated with constant control for
% each trajectory
% size of X         : 18 * (n_control x len(t_traj))
% U                 : random control inputs for each trajectory
% size of U         : 4 * (n_control x len(t_traj))
% 

addpath('nominal_pid/utils');
addpath('nominal_pid');

%% get robot params
params = get_params();


%% simulate a pid controller to follow waypoints
% states X = [x dx R wb]'
% initial condition is from rest (from ground)
X = []; X1=[]; X2=[];
U = []; U1=[]; U2=[];

for i=1:traj_param.n_traj
    % get control
    controlhandle = @controller;

    trajhandle = @traj_generator;
    if strcmp(traj_param.traj_type,'hover')
        height = traj_param.height(i);
        waypoints = zeros(3);
        waypoints(end,:) = linspace(0,height(i),3);
    end
    if strcmp(traj_param.traj_type,'circle')
        radius = traj_param.radius(i);
        direction = traj_param.direction;
        % direction = 1 for anticlockwise
        % direction = -1 for clockwise
        pres = 0.1;
        thetas = 0:pres:2*pi;
        % parametric expression for circle in XY-plane
        x_data = radius*cos(thetas);
        y_data = direction*radius*sin(thetas);
        waypoints = [x_data; y_data; zeros(size(x_data))];
    end
    trajhandle([],[],waypoints);

    % simulate ode
    % state - n x 13, with each row having format [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]
    % control - n x 4 with each row having format [f_next, M1, M2, M3]
    [t,x,u] = simulation_3d(trajhandle, controlhandle);
    % modify states for EDMD
    % X = [x dx R wb]'
    x_edmd = [x(:,1:3),x(:,23:25),x(:,14:22),x(:,11:13)];


    % collect data
    X = [X,x_edmd']; % [X(t1), X(t2), ..., X(tn)] stacked for each control input n
    U = [U, u'];

    % seperate data into snapshots
    X1 = [X1,x_edmd(1:end-1,:)'];
    X2 = [X2,x_edmd(2:end,:)'];
    U1 = [U1,u(1:end-1,:)'];
    U2 = [U2,u(2:end,:)'];
end


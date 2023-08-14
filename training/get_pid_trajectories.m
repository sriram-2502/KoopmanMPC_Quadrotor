function [T, X, U, X1, X2, U1, U2, traj_params] = get_pid_trajectories(traj_params,show_plot)
% function to random trajectories
% Inputs
% traj_params        : structure of required trajectory parameters
% Outputs
% X                 : random trajectories generated with constant control for  each trajectory
% size of X         : 18 * (n_control x len(t_traj))
% U                 : random control inputs for each trajectory
% size of U         : 4 * (n_control x len(t_traj))


addpath('nominal_pid/utils');
addpath('nominal_pid');
train_edmd = true; 

%% simulate a pid controller to follow waypoints
% states X = [x dx R wb]'

T = [];
X = []; X1=[]; X2=[];
U = []; U1=[]; U2=[];
X_pid = [];
traj_params.traj_len = [];

for i=1:traj_params.n_traj
    % get waypoints for each traj
    traj_params = traj_gen(traj_params,traj_params.params(i));
    trajhandle = @traj_generator;
    trajhandle([],[],traj_params.waypoints);

    % get control
    controlhandle = @controller;   

    % simulate ode
    % state - n x 13, with each row having format [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]
    % control - n x 4 with each row having format [f_next, M1, M2, M3]
    [t, x_pid, x_edmd, u] = simulation_3d(trajhandle, controlhandle, train_edmd,show_plot);
    
    % remove the first and the last parts of the trajcetory (to avoid rapid
    % changes in inputs)
    start_idx = 300; end_idx=length(t)-300;
    t = t(start_idx:end_idx);
    x_pid = x_pid(start_idx:end_idx,:);
    x_edmd = x_edmd(start_idx:end_idx,:);
    u = u(start_idx:end_idx,:);

    T = [T, t'];
    X_pid = [X_pid, x_pid'];
    traj_params.traj_len = [traj_params.traj_len; length(t)];
    if(train_edmd)  
        % collect data
        X = [X,x_edmd']; % [X(t1), X(t2), ..., X(tn)] stacked for each control input n
        U = [U, u'];
        % seperate data into snapshots
        X1 = [X1,x_edmd(1:end-1,:)'];
        X2 = [X2,x_edmd(2:end,:)'];
        U1 = [U1,u(1:end-1,:)'];
        U2 = [U2,u(2:end,:)'];
    else
        X = X_pid;
    end
end

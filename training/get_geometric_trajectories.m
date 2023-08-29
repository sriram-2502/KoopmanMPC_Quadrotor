function [T, X_edmd, U, X1, X2, U1, U2, traj_params] = get_geometric_trajectories(traj_params,show_plot,noise_flag,trun_traj)
% function to random trajectories
% Inputs
% traj_params        : structure of required trajectory parameters
% Outputs
% X                 : random trajectories generated with constant control for  each trajectory
% size of X         : 18 * (n_control x len(t_traj))
% U                 : random control inputs for each trajectory
% size of U         : 4 * (n_control x len(t_traj))


addpath('training/aux_functions');
addpath('training/test_functions');

%% simulate a geometric controller to follow waypoints
% states X = [x dx w R ei eI]'

T = [];
X_edmd = []; X1=[]; X2=[];
U = []; U1=[]; U2=[];
traj_params.traj_len = [];

if strcmp(traj_params.traj_type,'line')
    trajhandle = @command_line; 
elseif strcmp(traj_params.traj_type,'lissajous')
    trajhandle = @command_lissajous ;
end
controlhandle = @position_control;

param = get_params(noise_flag);
[k, param] = get_control_gains(param);

for i=1:traj_params.n_traj
    height = traj_params.params(i);
    param.height = height;
    % simulate ode
    % x_geometric = [xdot; vdot; Wdot; reshape(Rdot,9,1); ei_dot; eI_dot];
    % control - n x 4 with each row having format [f_net, M1, M2, M3]
    [t, x_geometric] = simulate_geometric(trajhandle,controlhandle,noise_flag,show_plot, height);
    % get control inputs for the trajectory
    u = get_geometric_control(t, x_geometric, trajhandle, controlhandle, k, param);

    % remove the first and the last parts of the trajcetory (to avoid rapid
    % changes in inputs)
    if trun_traj
        start_idx = 1; end_idx=length(t)-0;
        t = t(start_idx:end_idx);
        x_geometric = x_geometric(start_idx:end_idx,:);
        u = u(start_idx:end_idx,:);
    end

    % collect data
    T = [T, t'];
    traj_params.traj_len = [traj_params.traj_len; length(t)];
    X_edmd = [X_edmd,x_geometric(:,1:18)']; % ingnoring last six states ei, eI x_edmd = [x; dx; w; R(:)]
    U = [U, u'];
    % seperate data into snapshots
    X1 = [X1,x_geometric(1:end-1,:)'];
    X2 = [X2,x_geometric(2:end,:)'];
    U1 = [U1,u(1:end-1,:)'];
    U2 = [U2,u(2:end,:)'];
end

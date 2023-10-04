function [T, X_edmd, U, X1, X2, U1, UM1, traj_params] = get_geometric_trajectories(traj_params,show_plot,noise_flag,trun_traj,randomize_mass)
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
UM = []; UM1 = [];% mass as a control input

if strcmp(traj_params.traj_type,'line')
    trajhandle = @command_line; 
elseif strcmp(traj_params.traj_type,'lissajous')
    trajhandle = @command_lissajous;
elseif strcmp(traj_params.traj_type,'constVx')
    trajhandle = @command_constVx;
end
controlhandle = @position_control;

param = get_params(noise_flag);
[k, param] = get_control_gains(param);
original_mass = param.m;
param.initial_position = traj_params.initial_position;
param.position_stdev = traj_params.position_stdev;

for i=1:traj_params.n_traj
    % set the 'height' parameter
    param.height = traj_params.params(i);

    % change 'mass' parameter if 'randomize_mass' is true
    % select the mass from a uniform random distribution around param.m
    % (i.e. uniform random distribution around 2)
    if randomize_mass
        param.m = original_mass + 2*(rand(1)-0.5);
        fprintf('Randomized mass is: %2.4f \n',param.m)
    end
    
    % simulate ode
    % x_geometric = [xdot; vdot; Wdot; reshape(Rdot,9,1); ei_dot; eI_dot];
    % control - n x 4 with each row having format [f_net, M1, M2, M3]
    [t, x_geometric] = simulate_geometric(trajhandle,controlhandle,show_plot, k, param);
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
    UM = [UM, param.m*ones(size(u,1),1)'];
    % seperate data into snapshots
    X1 = [X1,x_geometric(1:end-1,:)'];
    X2 = [X2,x_geometric(2:end,:)'];
    U1 = [U1,u(1:end-1,:)'];
    U2 = [U2,u(2:end,:)'];
    UM1 = [UM1, param.m*ones(size(u,1)-1,1)'];
end

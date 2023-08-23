function [T, X_geometric, X, U, X1, X2, U1, U2, traj_params] = get_geometric_trajectories(traj_params,show_plot,flag)
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
train_edmd = true; 

%% simulate a pid controller to follow waypoints
% states X = [x dx R wb]'

T = [];
X = []; X1=[]; X2=[];
U = []; U1=[]; U2=[];
X_geometric =[];
traj_params.traj_len = [];

trajhandle = @command_line;
controlhandle = @position_control;

for i=1:traj_params.n_traj
    height = traj_params.params(i);
    % simulate ode
    % x_geometric = [xdot; vdot; Wdot; reshape(Rdot,9,1); ei_dot; eI_dot];
    % control - n x 4 with each row having format [f_net, M1, M2, M3]
    [t, x_geometric, x_edmd, u] = simulate_geometric(trajhandle,controlhandle,show_plot, height);
    
    x_geometric = parse_edmd_geometric(t, x_geometric);
    % remove the first and the last parts of the trajcetory (to avoid rapid
    % changes in inputs)
    start_idx = 1; end_idx=length(t)-0;
    t = t(start_idx:end_idx);
    x_geometric = x_geometric(start_idx:end_idx,:);
    x_edmd = x_edmd(start_idx:end_idx,:);
    u = u(start_idx:end_idx,:);

    T = [T, t'];
    X_geometric = [X_geometric, x_geometric'];
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

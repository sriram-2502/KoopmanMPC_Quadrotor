clc; clear; close all;
% import functions
addpath dynamics edmd mpc utils training
addpath(genpath('training'))
addpath('training/aux_functions');
addpath('training/test_functions');

dt_sim = 0.01;

%% get PID trajectories
% traj_params.traj_type = 'circle';%'hover';%'line';
% % parameter: 
% % hover:-> height
% % circle:-> radius
% % line:-> end point
% traj_params.params = linspace(0.5,1,1);  
% traj_params.n_traj = length(traj_params.params);
% flag='notraining';
% % get U from PID dynamics
% [T, X_PID, U, X1, X2, U1, U2, traj_params] = get_pid_trajectories(traj_params,show_plot,flag);
% 
% % simulate dynamics_SRB
% tstart = 0;
% tend = dt_sim;
% 
% quad_params = sys_params;
% Xt = X_PID(:,1); X_SRB =[];
% for ii = 1:length(T)
%     
%     Ut = U(:,ii);
%     [t,X] = ode45(@(t,x) dynamics_SRB(t, x, Ut, quad_params),[tstart,tend],Xt);
%     Xt = X(end,:)';
% 
%     tstart = tend;
%     tend = tstart + dt_sim;
% 
%     X_SRB = [X_SRB,X(end,:)'];
% end
% X_SRB = X_SRB';
% X_PID2SRB = parse_edmd(T,X_PID');

%% get geometric trajectories
traj_params.traj_type = 'line';%'hover';%'line';
% parameter: 
% hover:-> height
% circle:-> radius
% line:-> end point
traj_params.params = 2;  
traj_params.n_traj = length(traj_params.params);
flag='notraining';
show_plot = true;
% get U from PID dynamics
[T, X_geometric] = get_geometric_trajectories(traj_params,show_plot,flag);

% simulate dynamics_SRB
trajhandle = @command_line;
controlhandle = @position_control;
params = get_params();
[k, params] = get_control_gains(params);
params.height = 2;
X0 = [X_geometric(:,1);zeros(6,1)]'; 
t = 0:0.01:10;
[T_SRB, X_SRB] = ode45(@(t, XR) dynamics_SRB(t, XR, k, params, trajhandle, controlhandle), t, X0, ...
odeset('RelTol', 1e-6, 'AbsTol', 1e-6));
 
X_SRB = X_SRB';
X_geometric2SRB = parse_edmd_geometric(T,X_geometric');
X_PID2SRB = X_geometric2SRB;

%% plots
% parse each state for plotting
close all;
x_srb=[]; dx_srb = []; theta_srb =[]; wb_srb=[];
x_pid2srb=[]; dx_pid2srb = []; theta_pid2srb =[]; wb_pis2srb=[];
X_srb = []; X_pid2srb = [];
for i = 1:length(T)
    x_srb = [x_srb, X_SRB(i,1:3)']; 
    dx_srb = [dx_srb, X_SRB(i,4:6)']; 
    R_srb = reshape(X_SRB(i,7:15),[3,3]);
    theta_srb = [theta_srb, vee_map(logm(R_srb))]; 
    wb_srb = [wb_srb, X_SRB(i,16:18)']; 

    x_pid2srb = [x_pid2srb, X_PID2SRB(i,1:3)']; 
    dx_pid2srb = [dx_pid2srb, X_PID2SRB(i,4:6)']; 
    R_pid2srb = reshape(X_PID2SRB(i,7:15),[3,3]); %todo make R positive def in EDMD
    theta_pid2srb = [theta_pid2srb, vee_map(logm(R_pid2srb))]; 
    wb_pis2srb = [wb_pis2srb, X_PID2SRB(i,16:18)']; 
end

X_srb.x = x_srb; X_srb.dx = dx_srb;
X_srb.theta = theta_srb; X_srb.wb = wb_srb;

X_pid2srb.x = x_pid2srb; X_pid2srb.dx = dx_pid2srb;
X_pid2srb.theta = theta_pid2srb; X_pid2srb.wb = wb_pis2srb;

%state traj plots
fig_num = 10; flag = 'compare';
state_plots(fig_num,T,X_pid2srb,X_srb,flag)
fig_num = 20;

% control polts
control_plots(fig_num,T,U')

figure(1);
plot3(x_srb(1,:), x_srb(2,:), x_srb(3,:)); hold on
plot3(x_pid2srb(1,:), x_pid2srb(2,:), x_pid2srb(3,:), '--'); hold on
grid on; box on; axis square;
xlabel('$x_1$','FontSize',20, 'Interpreter','latex')
ylabel('$x_2$','FontSize',20, 'Interpreter','latex')
zlabel('$x_3$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
axes.LineWidth=2;
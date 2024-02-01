clc; clear; close all;

%set default figure properties
set(0,'DefaultLineLineWidth',2)
set(0,'defaultfigurecolor',[1 1 1])

% fix random seed
seed = 1;
rng(seed);

% import functions
addpath dynamics edmd mpc utils training
addpath(genpath('training'))


%% generate trajectory data using nominal controller
% current options are 'line','lissajous','constVx';
traj_params.traj_type = 'lissajous';
show_plot = false;
noise_flag = true;

% if trun_traj=true, need to set the indices inside 
% get_geometric_trajectories
trunc_traj = false; 

% train with mass as input if true, picks a mass value from from uniform 
% distribution about the nominal value of m=2
train_wt_random_mass = false;
mass_inv_as_param = false;
if train_wt_random_mass
%     mass_inv_as_param = true;       % if 1/mass is taken as input
    mass_inv_as_param = false;      % if mass is taken as input
end
% parameter: 
% hover:-> height
% circle:-> radius
% line:-> end point
initial_position = 1*[0,0,0]';
traj_params.initial_position = initial_position;
traj_params.position_stdev = 0.1*[1,1,1]';
traj_params.params = initial_position(1)+linspace(0.5,5,10);
traj_params.n_traj = length(traj_params.params);
[~, ~, ~, X1, X2, U1, UM1, traj_params] = get_geometric_trajectories(...
    traj_params,show_plot,noise_flag,trunc_traj,train_wt_random_mass);

%% get EDMD matrices
n_basis = 1; % n=3 works best
if train_wt_random_mass
    if mass_inv_as_param
        EDMD = get_EDMD(X1, X2, [U1;1./UM1], n_basis);
    else
        EDMD = get_EDMD(X1, X2, [U1;UM1], n_basis);
    end
else
    EDMD = get_EDMD(X1, X2, U1, n_basis);
end
A = EDMD.A;
B = EDMD.B;
C = EDMD.C;
Z1 = EDMD.Z1;
Z2 = EDMD.Z2;
EDMD.n_basis = n_basis;

close(figure(11111))
figure(11111)
viscircles([0,0],1)
hold on
scatter(real(eig(A)),imag(eig(A)))
axis tight
axis square
hold off

%% evaluate EDMD prediction for n timesteps
show_plot = true;
noise_flag = true;
trunc_traj = false;
test_wt_random_mass = false;
traj_params.params = traj_params.initial_position(1)+linspace(1.25,1.75,5);  
traj_params.n_traj = length(traj_params.params);
[~, X, ~, ~, ~, U1, UM1, traj_params] = get_geometric_trajectories(...
    traj_params,show_plot,noise_flag,trunc_traj,test_wt_random_mass);
EDMD.eval_horizon = 1000; % 'eval_horizon' step prediction
if train_wt_random_mass
    if mass_inv_as_param
        X_eval_std = eval_EDMD_pid(X,[U1;1./UM1],traj_params,EDMD,show_plot);
    else
        X_eval_std = eval_EDMD_pid(X,[U1;UM1],traj_params,EDMD,show_plot);
    end
else
    X_eval_std = eval_EDMD_pid(X,U1,traj_params,EDMD,show_plot);
end
% X_eval_mor = eval_EDMD_reducedOrder(X,U,traj_params,EDMD,show_plot);

%% do MPC
% 'get_geometric_trajectories' parameters
mpc_ref_wt_random_mass = false;
noise_flag = true;

% MPC parameters
mass_change_flag = false;
online_update_flag = true;
params = get_params(noise_flag);
EDMD.mass_as_input = train_wt_random_mass;
if EDMD.mass_as_input
    EDMD.mass_inv_as_param = mass_inv_as_param;
end

% set params
params.use_casadi = false;
params.predHorizon = 20;
params.simTimeStep = 1e-3;
dt_sim = params.simTimeStep;

% simulation time
if strcmp(traj_params.traj_type,'line')
    params.SimTimeDuration = 5;  % [sec]
elseif strcmp(traj_params.traj_type,'lissajous')
    params.SimTimeDuration = 3;  % [sec]
elseif strcmp(traj_params.traj_type,'constVx')
    params.SimTimeDuration = 5;
end
params.MAX_ITER = floor(params.SimTimeDuration/ params.simTimeStep);

% get reference trajectory
show_plot = false;
trunc_traj = false;
traj_params.initial_position = initial_position;%+4*(rand(3,1)-0.5);
traj_params.params = traj_params.initial_position(1)+1.5;  
traj_params.n_traj = length(traj_params.params);
[~, X_ref_mpc, ~, ~, ~, U1, UM1, ~] = get_geometric_trajectories(...
    traj_params,show_plot,noise_flag,trunc_traj,mpc_ref_wt_random_mass);

% reduce number of refernce points in the trajectory
% hold_for = 5;%mpc_params.predHorizon;
% remove_num = mod(size(X_ref,2),hold_for);
% X_ref_hold = zeros(size(X_ref,1),size(X_ref,2)-remove_num);
% for i=hold_for:hold_for:size(X_ref,2)
%     X_ref_hold(:,i-(hold_for-1):i) = repmat(X_ref(:,i),1,hold_for);
% end

% get lifted states
Z_ref = [];
X0 = X_ref_mpc(:,1);
X_ref_mpc = X_ref_mpc(:,2:end);
% X_ref = X_ref_hold(:,2:end);
for i = 1:length(X_ref_mpc) % compare n+1 timesteps
    x_des = X_ref_mpc(:,i);
    z = get_basis(x_des,n_basis);
    Z_ref = [Z_ref,z];
end
Z0 = get_basis(X0,n_basis);
EDMD.update_flag = false;
if train_wt_random_mass
    U_ref_mpc = [U1;UM1];
else
    U_ref_mpc = U1;
end

mpc = sim_MPC(EDMD,Z0,Z_ref,X_ref_mpc,params,U_ref_mpc,...
    mass_change_flag,online_update_flag);
mpc

%% MPC plots
% parse each state for plotting
close all
x_ref=[]; dx_ref = []; theta_ref =[]; wb_ref=[];
x_mpc=[]; dx_mpc = []; theta_mpc =[]; wb_mpc=[];
X_ref_plot = []; X_mpc = []; t_plot = [];
skip_idx=100; 
for i = 1:skip_idx:length(mpc.t)
    x_ref = [x_ref, mpc.X_ref(i,1:3)'];
    dx_ref = [dx_ref, mpc.X_ref(i,4:6)'];
    bRw_ref = reshape(mpc.X_ref(i,10:18),[3,3]);
    theta_ref = [theta_ref, vee_map(logm(bRw_ref))];
    wb_ref = [wb_ref, mpc.X_ref(i,7:9)'];

    % for prediction
    x_mpc = [x_mpc, mpc.X(i,1:3)'];
    dx_mpc = [dx_mpc, mpc.X(i,4:6)'];
    bRw = reshape(mpc.X(i,10:18),[3,3]);
    theta_mpc = [theta_mpc, vee_map(logm(bRw))];
    wb_mpc = [wb_mpc, mpc.X(i,7:9)'];

    t_plot = [t_plot, mpc.t(i)];
end

X_ref_plot.x = x_ref; X_ref_plot.dx = dx_ref;
X_ref_plot.theta = theta_ref; X_ref_plot.wb = wb_ref;

X_mpc.x = x_mpc; X_mpc.dx = dx_mpc;
X_mpc.theta = theta_mpc; X_mpc.wb = wb_mpc;

%state traj plots
fig_num = 10; flag = 'mpc';
state_plots(fig_num,t_plot,X_mpc,X_ref_plot,flag)
fig_num = 20;

% control polts
control_plots(fig_num,mpc.t,mpc.U,U_ref_mpc,mass_inv_as_param)

figure(1);
plot3(x_ref(1,:), x_ref(2,:), x_ref(3,:)); hold on
plot3(x_mpc(1,:), x_mpc(2,:), x_mpc(3,:), '--'); hold on
grid on; box on; axis square;
xlabel('$x_1$','FontSize',20, 'Interpreter','latex')
ylabel('$x_2$','FontSize',20, 'Interpreter','latex')
zlabel('$x_3$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
set(gca, 'YDir', 'reverse', 'ZDir', 'reverse');
axes.LineWidth=2;

%% mass plots
for i=1:size(mpc.estimated_mass,1)
    figure(110+i)
    plot(mpc.estimated_mass(i,:))
end

% for i=1:size(mpc.estimated_mass_edmd,1)
%     figure(220+i)
%     plot(mpc.estimated_mass_edmd(i,:))
% end
% 
% figure(300)
% plot(mean(mpc.estimated_mass_edmd([1,2,3,4,5,6,7,10,11,12,13,14,15,16,17],:)))
% 
% figure(301)
% plot(mean(mpc.estimated_mass_edmd([1,2,3,4,5,6],:)))
% 
% figure(302)
% plot(mean(mpc.estimated_mass_edmd([5,6],:)))
% 
% m_avg = mean(mpc.estimated_mass_edmd([1,2,3,4,5,6],:));
m_avg = mean(mpc.estimated_mass,1);
% windowSize = 500; 
% b = (1/windowSize)*ones(1,windowSize);
% a = 1;
% m_filtered = filter(b,a,m_avg);
% figure(303)
% plot(m_filtered)
% hold on

% m_avg = mean(mpc.estimated_mass,1);
figure(304)
plot(m_avg)
% mean(m_avg)
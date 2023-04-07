clc; clear all;
% This is the main file for generating/loading data

dt = 0.1;                    % delta_t for the Euler approximation of ode
x_lims = [-2,2];             % lower and upper limits of x-data for edmd
y_lims = [-2,2];             % lower and upper limits of y-data for edmd
% assuming x and y are reset to [0,0], every-time mpc is solved
theta_lims = [-0.35,0.35];   % lower and upper limits of \theta-data for edmd
v_lims = [0.8, 1.2];         % lower and upper limits of v-data for edmd

% input limits
u1_lim = [-0.2,0.2];            % u1=a
u2_lim = [-1.1,1.1];            % u2=tan(w)/wb

%% Generate data
n=4;m=2;                    % number of states and inputs
num_init = 1000;
X_data = [x_lims(1) + (x_lims(2)-x_lims(1)).*rand(1,num_init);
          y_lims(1) + (y_lims(2)-y_lims(1)).*rand(1,num_init);
          theta_lims(1) + (theta_lims(2)-theta_lims(1)).*rand(1,num_init);
          v_lims(1) + (v_lims(2)-v_lims(1)).*rand(1,num_init)];

U_data = [u1_lim(1) + (u1_lim(2)-u1_lim(1)).*rand(1,num_init);
          u2_lim(1) + (u2_lim(2)-u2_lim(1)).*rand(1,num_init)];

Y_data = vModel(X_data,U_data,dt);

%% Build dictionary
polyOrder = 2;
x = sym('x',[n,1],'real');
dumX = sym('dumX','real');
polyLift = 1;
for i=1:polyOrder
    polyLift = unique(kron(polyLift,x));
end
trigX3 = [sin(x(3));cos(x(3))];
x4TrigX3 = kron(x(4),trigX3);
dict = [polyLift;
        x4TrigX3];
liftFuns = matlabFunction(dict,'vars',{x});

%% Solve for A,B matrices
X_phi = liftFuns(X_data);
Y_phi = liftFuns(Y_data);

z_t_plus_1 = Y_phi;
z_t_U = [X_phi; U_data]';
A_B_mat = z_t_plus_1 * z_t_U * pinv(z_t_U'*z_t_U);
A_edmd = A_B_mat(:,1:end-m);
B_edmd = A_B_mat(:,end-m+1:end);

%% Test the Koopman linearized model
ni = 1000000;
X_d = [x_lims(1) + (x_lims(2)-x_lims(1)).*rand(1,ni);
       y_lims(1) + (y_lims(2)-y_lims(1)).*rand(1,ni);
       theta_lims(1) + (theta_lims(2)-theta_lims(1)).*rand(1,ni);
       v_lims(1) + (v_lims(2)-v_lims(1)).*rand(1,ni)];
X_d_lft = liftFuns(X_d);

U_d = [u1_lim(1) + (u1_lim(2)-u1_lim(1)).*rand(1,ni);
       u2_lim(1) + (u2_lim(2)-u2_lim(1)).*rand(1,ni)];

Y_d_model = vModel(X_d,U_d,dt);
% Y_d_lft = dictionary(Y_d_model,cardinality);

Y_d_koop = A_edmd*X_d_lft + B_edmd*U_d;

for i=1:10
    U_temp = [u1_lim(1) + (u1_lim(2)-u1_lim(1)).*rand(1,ni);
              u2_lim(1) + (u2_lim(2)-u2_lim(1)).*rand(1,ni)];
    Y_d_model = vModel(Y_d_model,U_temp,dt);
    Y_d_koop = A_edmd*Y_d_koop + B_edmd*U_temp;
end

Y_diff = abs(Y_d_model-Y_d_koop(1:4,:));
% Y_err = sqrt(sum(Y_diff.*Y_diff));
% max_Y_err = max(Y_err)
% mean_Y_err = mean(Y_err)
Y_err_max = max(Y_diff,[],2)
Y_err_min = min(Y_diff,[],2)
Y_err_mean = mean(Y_diff,2)

%%
%% Functions used
%%
function Y = vModel(X, U, dt)
% This file contains the model of the single-track vehicle
% states X:={x,y,v,\theta}
% control U:={a,w}
%parameters
% wb = 0.25;
% initialize the output matrix with zeros
Y = zeros(size(X));
Y(1,:) = X(1,:) + X(4,:).*cos(X(3,:)).*dt;
Y(2,:) = X(2,:) + X(4,:).*sin(X(3,:)).*dt;
Y(3,:) = X(3,:) + X(4,:).*U(2,:).*dt;  % wb is included in u2
Y(4,:) = X(4,:) + U(1,:).*dt;
end

%%%%
%%%%
function dXdt = dynamics_SRB(t,X,k,params,trajhandle,controlhandle)
% Inputs
% t         - simulation timestep
% X         - states for SRB dynamics
% U         - control inputs for SRB dynamics
% params    - physical paramters
%
% Ouputs
% dXdt      - future states based on SRB dynamics
%% parameters
mass = params.m;
J = params.J; % inertia tensor in body frame {B}
g = 9.81;
e3 = [0;0;1];

%% decompose
% states X = [x dx R wb]'
x = reshape(X(1:3),[3,1]); % pos in inertial frame
dx = reshape(X(4:6),[3,1]); % vel in inertial frame
wRb = reshape(X(7:15),[3,3]); % Rotation matrix body to world
wb = reshape(X(16:18),[3,1]); % ang vel in body frame
params.R = wRb;

% control inputs U = [Fb Mb]'
desired = trajhandle(t,params);
[fb, Mb, ei_dot, eI_dot, ~, ~] = controlhandle([X;zeros(6,1)], desired, k, params);

%% dynamics
% ddx = 1/mass * Fb * e3 - g*R'*e3; % net accleration
ddx = wRb'*(params.g * e3 - fb / mass * wRb * e3 + params.x_delta / mass); % net accleration
% dR = R * hat_map(wb); % update Rotation matrix
dR = wRb * hat_map(wb); % update Rotation matrix
% dwb = J \ (Mb - hat_map(wb) * J * wb);
dwb = J \ (-hat_map(wb) * J * wb + Mb + params.R_delta);

%% return states
dXdt = [dx;ddx;dR(:);dwb;ei_dot;eI_dot];

end








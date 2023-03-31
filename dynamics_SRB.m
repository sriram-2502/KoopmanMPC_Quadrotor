function dXdt = dynamics_SRB(t,X,U,params)
% Inputs
% t         - simulation timestep
% X         - states for SRB dynamics
% U         - control inputs for SRB dynamics
% params    - physical paramters
% params.m  - mass of the robot
% params.J  - inertial of the robot (world frame)
%
% Ouputs
% dXdt      - future states based on SRB dynamics

%% parameters
mass = params.mass;
J = params.J; % inertia tensor in body frame {B}
g = 9.81;
e3 = [0;0;1];

%% decompose
% states X = [x dx R wb]'
x = reshape(X(1:3),[3,1]); % pos in inertial frame
dx = reshape(X(4:6),[3,1]); % vel in inertial frame
R = reshape(X(7:15),[3,3]); % Rotation matrix
wb = reshape(Xt(16:18),[3,1]); % ang vel in body frame

% control inputs U = [Fb Mb]'
Fb = U(1); % net force in body frame
Mb = U(2:4); % Moments in body frame

%% dynamics
ddx = 1/mass * Fb * e3 - g*R'*e3; % net accleration
dR = R * hatMap(wb); % update Rotation matrix
dwb = J \ (Mb - hatMap(wb) * J * wb);

%% return states
dXdt = [dx;ddx;dR(:);dwb];

end








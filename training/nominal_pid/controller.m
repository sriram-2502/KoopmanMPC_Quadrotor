function [F, M] = controller(state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yaw_dot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Constants
% Kp1 = 5;
% Kd1 = 0.6;
% Kp3 = 10;
% Kd3 = 5;
% Kpa = 100;
% Kda = 1.2;
% Kp2 = Kp1;
% Kd2 = Kd1;
% Kpb = Kpa;
% Kpc = Kpb;
% Kdb = Kda;
% Kdc = Kdb;

% Kp1 = 12;   %10-15
% Kd1 = 2;    %1-3
% Kpa = 100;   %45-50
% Kda = 1;    %1-4
% Kpc = 30;    %1-4
% Kdc = 0.7;  %0.5-0.9
% Kp2 = Kp1;
% Kp3 = Kp2;
% Kd2 = Kd1;
% Kd3 = Kd2;
% Kpb = Kpa;
% Kdb = Kda;

% Kp1 = 180;  %180
% Kd1 = 35;   %35
% Kp3 = 90;   %90
% Kd3 = 22;   %22
% 
% Kpa = 100;  %100
% Kda = 1.68; %1.68
% 
% Kp2 = Kp1;
% Kd2 = Kd1;
% Kpb = Kpa;
% Kdb = Kda;
% Kpc = Kpb;
% Kdc = Kdb;
% 
% % Equation for the error goes to zero (11)
% r1des_ddot = des_state.acc(1) + Kd1*(des_state.vel(1)-state.vel(1)) + Kp1*(des_state.pos(1)-state.pos(1));
% r2des_ddot = des_state.acc(2) + Kd2*(des_state.vel(2)-state.vel(2)) + Kp2*(des_state.pos(2)-state.pos(2));
% r3des_ddot = des_state.acc(3) + Kd3*(des_state.vel(3)-state.vel(3)) + Kp3*(des_state.pos(3)-state.pos(3));
% 
% % equation 14
% % phi_des     = (r1des_ddot*sin(des_state.yaw)-r2des_ddot*cos(des_state.yaw))/params.mass;
% % theta_des   = (r1des_ddot*cos(des_state.yaw)+r2des_ddot*sin(des_state.yaw))/params.mass;
% phi_des     = (r1des_ddot*(des_state.yaw)-r2des_ddot)/params.mass;
% theta_des   = (r1des_ddot + r2des_ddot*des_state.yaw)/params.mass;
%     
% pdes = 0;
% qdes = 0;
% 
% % Thurst
% F = params.mass*(params.gravity+r3des_ddot);
% 
% % Moment
% M =[Kpa*(phi_des - state.rot(1))+Kda*(qdes - state.omega(1));
%     Kpb*(theta_des - state.rot(2))+Kdb*(pdes- state.omega(2));
%     Kpc*(des_state.yaw - state.rot(3))+Kdc*(des_state.yawdot- state.omega(3))];


Kpx = 180;
Kdx = 35;
Kpz = 90;
Kdz = 20;
Kpphi = 100;
Kdphi = 1.8;

Kpy = Kpx;
Kdy = Kdx;
Kppsi = Kpphi;
Kdpsi = Kdphi;
Kptheta = Kpphi;
Kdtheta = Kdphi;


r1des_ddot = des_state.acc(1)+Kdx*(des_state.vel(1)-state.vel(1))+Kpx*(des_state.pos(1)-state.pos(1));
r2des_ddot = des_state.acc(2)+Kdy*(des_state.vel(2)-state.vel(2))+Kpy*(des_state.pos(2)-state.pos(2));
r3des_ddot = des_state.acc(3)+Kdz*(des_state.vel(3)-state.vel(3))+Kpz*(des_state.pos(3)-state.pos(3));

phi_des   	= (r1des_ddot*des_state.yaw - r2des_ddot)/params.gravity ;
theta_des 	= (r1des_ddot + r2des_ddot*des_state.yaw)/params.gravity ;

M =[Kpphi*(phi_des-state.rot(1)) + Kdphi*(0-state.omega(1));
	Kptheta*(theta_des-state.rot(2)) + Kdtheta*(0-state.omega(2));
	Kppsi*(des_state.yaw-state.rot(3)) + Kdpsi*(des_state.yawdot-state.omega(3))];

F = params.mass*(params.gravity+r3des_ddot);


end

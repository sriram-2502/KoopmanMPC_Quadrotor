function [ s ] = init_state_edmd(start, yaw )
%INIT_STATE Initialize 13 x 1 state vector

s     = zeros(13,1);
phi0   = 0.0;
theta0 = 0.0;
psi0   = yaw;
Rot0   = RPYtoRot_ZXY(phi0, theta0, psi0);
Quat0  = RotToQuat(Rot0);

s(1)  = start(1); %x
s(2)  = start(2); %y
s(3)  = start(3); %z
s(4)  = 0;        %xdot
s(5)  = 0;        %ydot
s(6)  = 0;        %zdot
bRw = Rot0; % Rotation matrix
wRb = bRw';
s(7:15) = bRw(:); %R
s(16) = 0;        %p
s(17) = 0;        %q
s(18) = 0;        %r

end
function [x_edmd, usave] = parse_edmd(tsave, xsave, controlhandle, trajhandle, params)
%function [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, controlhandle, trajhandle, params), timeint, x)
% inputs: 
% f_handle- function handle for dynamics
% timeint - vector for time span or simulation time
% x0 - initial condition

parse_control = true;
if nargin<3
    parse_control = false;
    controlhandle=[];
    trajhandle = [];
    params = [];
end

% [tsave, xsave] = ode45(f_handle, timeint, x0);
x_edmd = [];
usave = [];

for i=1:length(tsave) 
    if parse_control
        % get control outputs
        current_state = stateToQd(xsave(i,:));
        desired_state = trajhandle(tsave(i), current_state);
        [F, M] = controlhandle(current_state, desired_state, params);
        % assemble u
        u = [F; M];
        usave = [usave, u(:)];
    end

    % add rotation matrix states
    qW = xsave(i,7);
    qX = xsave(i,8);
    qY = xsave(i,9);
    qZ = xsave(i,10);
    quat = [qW; qX; qY; qZ];
    bRw = QuatToRot(quat);
    wRb = bRw';

    % x_edmd = [x, xdot_body, R(:), wb] -- 18 states
    % bRw gives best prediction
    % use bRw to take acceleration from world frame to body frame for EDMD 
    accel_body = bRw*xsave(i,4:6)'; 
    x_edmd = [x_edmd; [xsave(i,1:3),accel_body',wRb(:)',xsave(i,end-2:end)]];
end

usave = usave';

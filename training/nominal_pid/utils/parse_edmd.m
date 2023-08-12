function [tsave, xsave, usave, x_edmd] = parse_edmd(f_handle, timeint, x0, controlhandle, trajhandle, params)
%function [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, controlhandle, trajhandle, params), timeint, x)
% inputs: 
% f_handle- function handle for dynamics
% timeint - vector for time span or simulation time
% x0 - initial condition

[tsave, xsave] = ode45(f_handle, timeint, x0);
x_edmd = [];
usave = [];

for i=1:length(tsave) 
    % get control outputs
    current_state = stateToQd(xsave(i,:));
    desired_state = trajhandle(tsave(i), current_state);
    [F, M] = controlhandle(current_state, desired_state, params);
    % assemble u
    u = [F; M];
   
    % add rotation matrix states
    qW = xsave(i,7);
    qX = xsave(i,8);
    qY = xsave(i,9);
    qZ = xsave(i,10);
    quat = [qW; qX; qY; qZ];
    bRw = QuatToRot(quat);
    wRb = bRw';

    % update states
    % get RK4
%     k1 = f_handle(timeint(i),x);
%     k2 = f_handle(timeint(i)+dt/2,x+k1*dt/2);
%     k3 = f_handle(timeint(i)+dt/2,x+k2*dt/2);
%     k4 = f_handle(timeint(i)+dt,x+k3*dt);
%     x = x + dt/6*(k1+2*k2+2*k3+k4);
%     x = x + dt*f_handle(timeint(i),x); % euler

    % x_edmd = [x, xdot_body, R(:), wb] -- 18 states
    accel_body = bRw*xsave(i,4:6)';
    x_edmd = [x_edmd; [xsave(i,1:3),accel_body',wRb(:)',xsave(i,end-2:end)]];
    usave = [usave, u(:)];
    
end

usave = usave';
tsave = timeint';
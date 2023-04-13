function [A_lin, B_lin] = linearize_SRB()

x = sym('x',[18,1]);
u = sym('u',[4,1]);
t = 0;
params = get_params();
f = dynamics_SRB(t,x,u,params);

x0 = zeros(18,1);
u0 = [0;0;0;0];

A_lin = double(subs(jacobian(f,x),x,x0)); 
B_lin = double(subs(jacobian(f,u),u,u0));
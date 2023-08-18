function state_plots(fig_num,tout,X,X_ref,flag)
figure(fig_num)
SimTimeDuration = tout(end);

%parse states
x = X.x; dx = X.dx;
theta = X.theta; wb = X.wb;
x_ref = X_ref.x; dx_ref = X_ref.dx;
theta_ref = X_ref.theta; wb_ref = X_ref.wb;

%linear states
subplot(6,4,1)
plot(tout, x_ref(1,:)); hold on;
plot(tout, x(1,:),'--'); hold on;
axes = gca;
set(axes,'FontSize',15);
ylabel('$x$','FontSize',20, 'Interpreter','latex')
box on; axes.LineWidth=2;
if(strcmp(flag,'mpc'))
    lgd = legend('reference','MPC');
elseif (strcmp(flag,'compare'))
    lgd = legend('pid2srb','srb');
else
    lgd = legend('true','predicted');
end
lgd.Location = 'best';
lgd.NumColumns = 2;
xlim([0,SimTimeDuration]);

subplot(6,4,5)
plot(tout, x_ref(2,:)); hold on; 
plot(tout, x(2,:),'--'); hold on;
ylabel('$y$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;
xlim([0,SimTimeDuration]);

subplot(6,4,9)
plot(tout, x_ref(3,:)); hold on;
plot(tout, x(3,:),'--'); hold on;
ylabel('$z$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;
xlim([0,SimTimeDuration]);

subplot(6,4,13)
plot(tout, dx_ref(1,:)); hold on;
plot(tout, dx(1,:),'--'); hold on;
ylabel('$v_x$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;
xlim([0,SimTimeDuration]);

subplot(6,4,17)
plot(tout, dx_ref(2,:)); hold on;
plot(tout, dx(2,:),'--'); hold on;
ylabel('$v_y$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;
xlim([0,SimTimeDuration]);

subplot(6,4,21)
plot(tout, dx_ref(3,:)); hold on; 
plot(tout, dx(3,:),'--'); hold on;
xlabel('$t$ (s)','FontSize',20, 'Interpreter','latex')
ylabel('$v_z$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;
xlim([0,SimTimeDuration]);

%angular states
subplot(6,4,2)
plot(tout, theta_ref(1,:)); hold on; 
plot(tout, theta(1,:),'-'); hold on;
ylabel('$\theta$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;
xlim([0,SimTimeDuration]);

subplot(6,4,6)
plot(tout, theta_ref(2,:)); hold on; 
plot(tout, theta(2,:),'-'); hold on;
ylabel('$\phi$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;
xlim([0,SimTimeDuration]);

subplot(6,4,10)
plot(tout, theta_ref(3,:)); hold on; 
plot(tout, theta(3,:),'-'); hold on;
ylabel('$\psi$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;
xlim([0,SimTimeDuration]);

subplot(6,4,14)
plot(tout, wb_ref(1,:)); hold on;
plot(tout, wb(1,:),'-'); hold on;
ylabel('$\omega_x$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;
xlim([0,SimTimeDuration]);

subplot(6,4,18)
plot(tout, wb_ref(2,:)); hold on;
plot(tout, wb(2,:),'-'); hold on;
ylabel('$\omega_y$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;
xlim([0,SimTimeDuration]);

subplot(6,4,22)
plot(tout, wb_ref(3,:)); hold on;
plot(tout, wb(3,:),'-'); hold on;
xlabel('$t$ (s)','FontSize',20, 'Interpreter','latex')
ylabel('$\omega_z$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;
xlim([0,SimTimeDuration]);
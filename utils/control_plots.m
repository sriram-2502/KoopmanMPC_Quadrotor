function control_plots(fig_num,tout,U)
SimTimeDuration = tout(end);
%plot control inputs
figure(fig_num)
subplot(4,4,1)
plot(tout,U(:,1)); hold on;
xlabel('$t$ (s)','FontSize',20, 'Interpreter','latex')
ylabel('$f_t$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;
xlim([0,SimTimeDuration]);

subplot(4,4,5)
plot(tout,U(:,2)); hold on;
xlabel('$t$ (s)','FontSize',20, 'Interpreter','latex')
ylabel('$M_1$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;
xlim([0,SimTimeDuration]);

subplot(4,4,9)
plot(tout,U(:,3)); hold on;
xlabel('$t$ (s)','FontSize',20, 'Interpreter','latex')
ylabel('$M_2$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;
xlim([0,SimTimeDuration]);

subplot(4,4,13)
plot(tout,U(:,4)); hold on;
xlabel('$t$ (s)','FontSize',20, 'Interpreter','latex')
ylabel('$M_3$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;
xlim([0,SimTimeDuration]);
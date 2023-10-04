function control_plots(fig_num,tout,U,U_ref_mpc,mass_inv_as_param)
SimTimeDuration = tout(end);
tMPC = 0.001*(1:size(U_ref_mpc,2));
%plot control inputs
figure(fig_num)
subplot(4,4,1)
plot(tout,U(:,1),'k'); hold on;
ylabel('$f_t$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;
xlim([0,SimTimeDuration]);
plot(tMPC,U_ref_mpc(1,:),'--r');

subplot(4,4,2)
plot(tout,U(:,2),'k'); hold on;
ylabel('$M_2$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;
xlim([0,SimTimeDuration]);
plot(tMPC,U_ref_mpc(2,:),'--r');

subplot(4,4,5)
plot(tout,U(:,3),'k'); hold on;
ylabel('$M_2$','FontSize',20, 'Interpreter','latex')
xlabel('$t$ (s)','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;
xlim([0,SimTimeDuration]);
plot(tMPC,U_ref_mpc(3,:),'--r');

subplot(4,4,6)
plot(tout,U(:,4),'k'); hold on;
xlabel('$t$ (s)','FontSize',20, 'Interpreter','latex')
ylabel('$M_3$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;
xlim([0,SimTimeDuration]);
plot(tMPC,U_ref_mpc(4,:),'--r');

if size(U,2)==5
    subplot(4,4,11)
    plot(tout,U(:,5),'k'); hold on;
    xlabel('$t$ (s)','FontSize',20, 'Interpreter','latex')
    if mass_inv_as_param
        plot(tMPC,1./U_ref_mpc(5,:),'--r');
        ylabel('$\frac{1}{Mass}$','FontSize',20, 'Interpreter','latex')
    else
        plot(tMPC,U_ref_mpc(5,:),'--r');
        ylabel('$Mass$','FontSize',20, 'Interpreter','latex')
    end
    axes = gca; set(axes,'FontSize',15);
    box on; axes.LineWidth=2;
    xlim([0,SimTimeDuration]);
end
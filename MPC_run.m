clear
close all

% Sample time
Ts = 0.01;

% Initialize state and input
qqval = zeros(7,1);
uuval = zeros(2,1);

%% Get ref trajectory for periodic eight
rad = 100; % radius in m
vel = 5; % velocity in m/s
tspan = 4*2*pi*rad/vel; % 2 cycles
[xref,yref] = gettraj(vel,rad,tspan);

%% to store
steps = numel(0:Ts:tspan);
timesteps = 0:Ts:tspan;
qqData=zeros(7,steps-100);
uuData=zeros(2,steps-100);

%% MPC Loop

% set the reference trajectory. The reference trajectory for rest 5 is
% given 0, but their weights are taken very small.
ref = [xref;yref;zeros(5,length(xref))];

for ii=1:steps-100
    if rem(ii,100) == 0
        disp(ii+" steps of "+steps+" complete.")
    end
    
    % linearization and discretization 
    Ad = eye(7)+Ts*AAVal_sym(qqval,uuval);
    Bd = Ts*BBVal_sym(qqval,uuval);
    Dd = Ts*(CombVal_sym(qqval,uuval)-AAVal_sym(qqval,uuval)*qqval-BBVal_sym(qqval,uuval)*uuval);
    
    X_in = qqval;       %Start point 
        
    hor=20;            % Horizon
    path2 = [ref(:,ii+1:1:ii+hor)];
    Q = diag([1000 1000 0.001 0.001 0.001 0.001 0.001]);
    P = diag([400 400 0.001 0.001 0.001 0.001 0.001]);                               % final state error weight
    R=1*eye(2);

    uMin=[-100;-10];      % Input cons
    uMax=[100;10];

    xx_lb =[-300;-200;-50;-20;-20;-20;-20]; % state constraints
    xx_ub = [300;200;50;20;20;10;10];


    Input = MPC_Car(Ad,Bd,P,Q,R,hor,X_in,Dd,path2, uMax, uMin, xx_lb, xx_ub); %get u(t)

    % non linear dynamics to update state using ode45
    [~,qqall] = ode45(@(t,q) CombVal_sym(qqval,Input),[timesteps(ii) timesteps(ii+1)], qqval);
    qqval = qqall(end,:)'; % updated state: x(t+1), becomes x(t) in next loop

    uuval=Input; %store for next cycle

    uuData(:,ii) = Input;
    qqData(:,ii) = qqval;

end

disp("Complete!")
%% deviation

dev = [xref(1:steps-100)-qqData(1,:);yref(1:steps-100)-qqData(2,:)];

max_dev = max(dev, [], 2);

if max(max_dev)>10
    disp("Tracking not properly")
else
    disp("Tracking is fine")
end

%% Create plots
FS = 12;
figure
plot(xref(1:length(qqData)),yref(1:length(qqData)),'LineWidth',1.5,'Color','b');
hold on
plot(qqData(1,:),qqData(2,:),'LineWidth',1.5,'Color','r','LineStyle','--');
ylabel('Y (m)','Interpreter','latex','FontSize',FS)
xlabel('X (m)','interpreter','latex','FontSize',FS)
legend('Reference','Actual','interpreter','latex')
title('Tarjectory Tracking with MPC','Interpreter','latex','FontSize',FS)
xlim([-250,250])
ax = gca;
ax.FontSize = FS;
%%
figure
subplot(221)
plot(timesteps(1:length(qqData)),xref(1:length(qqData)),'LineWidth',1.5,'Color','b');
hold on
plot(timesteps(1:length(qqData)),qqData(1,:),'LineWidth',1.5,'Color','r','LineStyle','--');
ylabel('$X_{ac}$ vs $X_{ref}$','Interpreter','latex','FontSize',FS)
xlabel('Time (s)','interpreter','latex','FontSize',FS)
legend('$X_{ref}$','$X_{ac}$','interpreter','latex')
title('X vs X Ref','Interpreter','latex','FontSize',FS)
xlim([0 tspan])
ax = gca;
ax.FontSize = 20;
subplot(222)
plot(timesteps(1:length(qqData)),yref(1:length(qqData)),'LineWidth',1.5,'Color','b');
hold on
plot(timesteps(1:length(qqData)),qqData(2,:),'LineWidth',1.5,'Color','r','LineStyle','--');
ylabel('$Y_{ac}$ vs $Y_{ref}$','Interpreter','latex','FontSize',FS)
xlabel('Time (s)','interpreter','latex','FontSize',FS)
legend('$Y_{ref}$','$Y_{ac}$','interpreter','latex')
title('Y vs Y Ref','Interpreter','latex','FontSize',FS)
xlim([0 tspan])
ax = gca;
ax.FontSize = 20;
% Plot of Deviation
subplot(223)
plot(timesteps(1:length(qqData)),qqData(1,:)-xref(1:length(qqData)),'LineWidth',1.5,'Color','b','LineStyle','--');
hold on
plot(timesteps(1:length(qqData)),10*ones(length(qqData),1),'LineWidth',1,'Color','r')
plot(timesteps(1:length(qqData)),-10*ones(length(qqData),1),'LineWidth',1,'Color','r')
ylabel('$\Delta X$','Interpreter','latex','FontSize',FS)
xlabel('Time (s)','interpreter','latex','FontSize',FS)
title('Error in X','Interpreter','latex','FontSize',FS)
xlim([0 tspan])
ylim([-12 12])
ax = gca;
ax.FontSize = 20;
subplot(224)
plot(timesteps(1:length(qqData)),qqData(2,:)-yref(1:length(qqData)),'LineWidth',1.5,'Color','b','LineStyle','--');
hold on
plot(timesteps(1:length(qqData)),10*ones(length(qqData),1),'LineWidth',1,'Color','r')
plot(timesteps(1:length(qqData)),-10*ones(length(qqData),1),'LineWidth',1,'Color','r')
ylabel('$\Delta Y$','Interpreter','latex','FontSize',FS)
xlabel('Time (s)','interpreter','latex','FontSize',FS)
title('Error in Y','Interpreter','latex','FontSize',FS)
xlim([0 tspan])
ylim([-12 12])
ax = gca;
ax.FontSize = 20;
%%
% Plot control input trajectories

figure
subplot(211)
plot(timesteps(1:length(qqData)),uuData(1,:),'LineWidth',2)
hold on
plot(timesteps(1:length(qqData)),uMax(1)*ones(length(qqData),1),'LineWidth',1,'Color','r')
plot(timesteps(1:length(qqData)),uMin(1)*ones(length(qqData),1),'LineWidth',1,'Color','r')
title('Control Input: $F_{x}$','Interpreter','latex','FontSize',FS)
ylabel('Force $F_{x}$ (N)','Interpreter','latex','FontSize',FS)
xlabel('Time (s)','interpreter','latex','FontSize',FS)
xlim([0 100])
ax = gca;
ax.FontSize = FS;
subplot(212)
plot(timesteps(1:length(qqData)),uuData(2,:),'LineWidth',2)
hold on
plot(timesteps(1:length(qqData)),uMax(2)*ones(length(qqData),1),'LineWidth',1,'Color','r')
plot(timesteps(1:length(qqData)),uMin(2)*ones(length(qqData),1),'LineWidth',1,'Color','r')
title('Control Input: Velocity of steering angle','Interpreter','latex','FontSize',FS)
ylabel('$\Delta\delta$','Interpreter','latex','FontSize',FS)
xlabel('Time (s)','interpreter','latex','FontSize',FS)
xlim([0 100])
ax = gca;
ax.FontSize = FS;



%% create animation

%Animate_Car(qqData,timesteps(1:length(qqData)),xref(1:length(qqData)),yref(1:length(qqData)))


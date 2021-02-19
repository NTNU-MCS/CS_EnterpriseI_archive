bag = rosbag('2021-01-21-03-47-57.bag');
bagselect = select(bag, 'Topic', '/qualisys/CSEI/odom'); 
bag_obs = select(bag, 'Topic', 'CSEI/observer/odom'); 
bag_joy = select(bag,'Topic','/joy');
bag_u = select(bag,'Topic','/CSEI/u');

ts = timeseries(bagselect, 'Twist.Twist.Linear.X','Twist.Twist.Linear.Y', 'Twist.Twist.Angular.Z', 'Pose.Pose.Position.X', 'Pose.Pose.Position.Y', 'Pose.Pose.Position.Z', 'Pose.Pose.Orientation.W', 'Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y','Pose.Pose.Orientation.Z');
ts_obs = timeseries(bag_obs, 'Twist.Twist.Linear.X','Twist.Twist.Linear.Y', 'Twist.Twist.Angular.Z', 'Pose.Pose.Position.X', 'Pose.Pose.Position.Y', 'Pose.Pose.Position.Z', 'Pose.Pose.Orientation.W', 'Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y','Pose.Pose.Orientation.Z');
%%
msgStructs = readMessages(bag_joy,'DataFormat','struct');
Axes = cellfun(@(m) double(m.Axes),msgStructs,'UniformOutput',false);
Buttons = cellfun(@(m) double(m.Buttons),msgStructs,'UniformOutput',false);
m1 = size(Axes,1);
m2 = size(Buttons,1);

inputStructs = readMessages(bag_u,'DataFormat','struct');
inputData = cellfun(@(m) double(m.Data),inputStructs,'UniformOutput',false);
m3 = size(inputData,1);
inputData = cell2mat(inputData);
inputData = reshape(inputData,[5,m3]);


ts_u = bag_u.timeseries;
time_u = ts_u.time - ts.Time(1);

ts_joy = bag_joy.timeseries;
time_joy = ts_joy.time - ts.Time(1);



time = ts.Time(:);
time = time-time(1);
vel  = ts.Data(:,1:3);
pos  = ts.Data(:,4:6);
quat = ts.Data(:,7:10);

eul = quat2eul(quat,'XYZ');
psi = 180/pi.*eul(:,3);

time_obs = ts_obs.Time(:);
time_obs = time_obs - ts.Time(1);
vel_obs  = ts_obs.Data(:,1:3);
pos_obs  = ts_obs.Data(:,4:6);
quat_obs = ts_obs.Data(:,7:10);

eul_obs = quat2eul(quat_obs,'XYZ');
psi_obs = 180/pi.*eul_obs(:,3);

Axes = cell2mat(Axes);
Axes = reshape(Axes,[6,m1]);
Buttons = cell2mat(Buttons);
Buttons = reshape(Buttons,[18,m2]);


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% PLOTTING

figure(421)
clf(421)
plot(time_joy,Axes(5,:))

figure(422)
clf(422)
plot(time_joy,Buttons)

figure(420)
clf(420)
hold on
plot(pos(:,2),pos(:,1),'linewidth', 1.5)
plot(pos_obs(:,2),pos_obs(:,1),'linewidth', 1.5)
ylabel('North [m]','interpreter','latex', 'FontSize', 13)
xlabel('East [m]','interpreter','latex', 'FontSize', 13)
legend({'$p$', '$p_{d}$'},'Interpreter','latex','Location','best','NumColumns',1, 'FontSize', 11)

%%
figure(1)
clf(1)
subplot(3,1,1)

hold on
plot(time,pos(:,1),'LineWidth', 1.5);
plot(time_obs,pos_obs(:,1),'--', 'LineWidth',1);
legend({'$x$', '$x_{d}$'},'Interpreter','latex','Location','best','NumColumns',1, 'FontSize', 11)
ylabel('Position [m]','interpreter','latex', 'FontSize', 13)
xlabel('Time [s]','interpreter','latex', 'FontSize', 13);

% figure(2)
% clf(2)
subplot(3,1,2)

hold on
plot(time,pos(:,2),'LineWidth',1.5);
plot(time_obs,pos_obs(:,2),'--', 'LineWidth',1);
legend({'$y$', '$y_{d}$'},'Interpreter','latex','Location','best','NumColumns',1, 'FontSize', 11)
ylabel('Position [m]','interpreter','latex', 'FontSize', 13)
xlabel('Time [s]','interpreter','latex', 'FontSize', 13);

subplot(3,1,3)
hold on
plot(time,psi,'LineWidth', 1.5);
plot(time_obs,psi_obs,'--', 'LineWidth',1);
legend({'$\psi$', '$\psi_{d}$'},'Interpreter','latex','Location','best','NumColumns',1,'fontsize',13)
xlabel('Time [s]','interpreter','latex','fontsize',13);
ylabel('Angle [deg]','interpreter','latex','fontsize',13)

figure(4)
clf(4)
subplot(3,1,1)
hold on

plot(time,vel(:,1),'LineWidth', 1.5);
plot(time_obs,vel_obs(:,1),'LineWidth', 1.5);
legend({'$u$', '$u_{d}$'},'Interpreter','latex','Location','best','NumColumns',1,'fontsize',13)
xlabel('Time [s]','interpreter','latex','fontsize',13);
ylabel('Angular velocity','interpreter','latex','fontsize',13)

subplot(3,1,2)
hold on

plot(time,vel(:,2),'LineWidth', 1.5);
plot(time_obs,vel_obs(:,2),'LineWidth', 1.5);
legend({'$v$', '$v_{d}$'},'Interpreter','latex','Location','best','NumColumns',1,'fontsize',13)
xlabel('Time [s]','interpreter','latex','fontsize',13);
ylabel('Angular velocity','interpreter','latex','fontsize',13)

subplot(3,1,3)
hold on
plot(time,vel(:,3),'LineWidth', 1.5);
plot(time_obs,vel_obs(:,3),'LineWidth', 1.5);
legend({'$r$', '$r_{d}$'},'Interpreter','latex','Location','best','NumColumns',1,'fontsize',13)
xlabel('Time [s]','interpreter','latex','fontsize',13);
ylabel('Angular velocity','interpreter','latex','fontsize',13)






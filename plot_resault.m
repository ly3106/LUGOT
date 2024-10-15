figure
x = position(:,2);
y = position(:,3);
plot(x,y,'o-','DisplayName','position')
xlabel('x (m)');
ylabel('y (m)');
legend show;

figure
x = uwb_global_lidar(:,1);
y = uwb_global_lidar(:,2);
plot(x,y,'DisplayName','uwb\_global\_lidar')
axis equal; % 设置x轴和y轴的比例尺相同
axis tight; % 调整坐标轴的范围以紧凑地显示图形
xlabel('x (m)');
ylabel('y (m)');
legend show

hold on;
x = UWB_KF.signals.values(:,1);
y = UWB_KF.signals.values(:,2);
plot(x,y,'.-','DisplayName','UWB\_KF')

x = UWB_EKF.signals.values(:,1);
y = UWB_EKF.signals.values(:,2);
plot(x,y,'.-','DisplayName','UWB\_EKF')

x = UWB_UKF.signals.values(:,1);
y = UWB_UKF.signals.values(:,2);
plot(x,y,'.-','DisplayName','UWB\_UKF')

x = UWB_PF.signals.values(:,1);
y = UWB_PF.signals.values(:,2);
plot(x,y,'.-','DisplayName','UWB\_PF')

hold off;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 绘制第二张主要对比图
figure
x = uwb_global_lidar(:,1);
y = uwb_global_lidar(:,2);
plot(x,y,'DisplayName','uwb\_global\_lidar')
axis equal; % 设置x轴和y轴的比例尺相同
axis tight; % 调整坐标轴的范围以紧凑地显示图形
xlabel('x (m)');
ylabel('y (m)');
legend show

hold on;

x = UWB_EKF_Rawd_CT_3D.signals.values(:,1);
y = UWB_EKF_Rawd_CT_3D.signals.values(:,3);
plot(x,y,'.-','DisplayName','UWB\_EKF\_Rawd\_CT\_3D')

x = UWB_UKF_Rawd_CT_3D.signals.values(:,1);
y = UWB_UKF_Rawd_CT_3D.signals.values(:,3);
plot(x,y,'.-','DisplayName','UWB\_UKF\_Rawd\_CT\_3D')
hold off;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 绘制第三张对比图，主要对比剔除异常点效果
figure
x = uwb_raw_global_lidar(:,1);
y = uwb_raw_global_lidar(:,2);
plot(x, y,'-','DisplayName','uwb\_raw\_global\_lidar')
hold on;
scatter(x(~valid_raw),y(~valid_raw), 'DisplayName','Non-triangular')
axis equal; % 设置x轴和y轴的比例尺相同
axis tight; % 调整坐标轴的范围以紧凑地显示图形
xlabel('x (m)');
ylabel('y (m)');
legend show

enableKF = Enable_KF.signals.values;
scatter(x(~enableKF),y(~enableKF), 'd','DisplayName','Eliminated')

x = uwb_global_lidar(:,1);
y = uwb_global_lidar(:,2);
plot(x,y,'DisplayName','uwb\_global\_lidar')

x = UWB_EKF_Rawd_CT_3D.signals.values(:,1);
y = UWB_EKF_Rawd_CT_3D.signals.values(:,3);
plot(x,y,'.-','DisplayName','UWB\_EKF\_Rawd\_CT\_3D')

x = UWB_UKF_Rawd_CT_3D.signals.values(:,1);
y = UWB_UKF_Rawd_CT_3D.signals.values(:,3);
plot(x,y,'.-','DisplayName','UWB\_UKF\_Rawd\_CT\_3D')

x = UWB_EKF_Rawd_CT_3D.signals.values(:,1);
y = UWB_EKF_Rawd_CT_3D.signals.values(:,3);
plot(x(enableKF),y(enableKF),'.-','DisplayName','UWB\_EKF\_Rawd\_CT\_3D\_Eliminated')

x = UWB_UKF_Rawd_CT_3D.signals.values(:,1);
y = UWB_UKF_Rawd_CT_3D.signals.values(:,3);
plot(x(enableKF),y(enableKF),'.-','DisplayName','UWB\_UKF\_Rawd\_CT\_3D\_Eliminated')

hold off;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 绘制第四张对比图，用于输出到论文
figure
x = uwb_raw_global_lidar(:,1);
y = uwb_raw_global_lidar(:,2);
plot(x, y,'-','DisplayName','uwb\_raw\_global\_lidar')
hold on;
scatter(x(~valid_raw),y(~valid_raw), 'DisplayName','Non-triangular')
axis equal;
axis tight;
xlabel('x (m)');
ylabel('y (m)');
% legend show
legend('NumColumns', 2, 'Location', 'northoutside'); 

enableKF = Enable_KF.signals.values;
scatter(x(~enableKF),y(~enableKF), 'd','DisplayName','Eliminated')

marker_interval = 800; % 每隔800个点显示一个标记
x = uwb_global_lidar(:,1);
y = uwb_global_lidar(:,2);
plot(x,y,'.-','DisplayName','uwb\_global\_lidar', 'MarkerSize',4, 'MarkerIndices', 1:marker_interval:length(fitted_x1))

x = UWB_EKF_Rawd_CT_3D_All.signals.values(:,1);
y = UWB_EKF_Rawd_CT_3D_All.signals.values(:,3);
plot(x,y,'.-','DisplayName','UWB_EKF_Rawd_CT_3D_All', 'MarkerSize',4, 'MarkerIndices', floor(marker_interval/3)+1:marker_interval:length(fitted_x2))

x = UWB_UKF_Rawd_CT_3D_All.signals.values(:,1);
y = UWB_UKF_Rawd_CT_3D_All.signals.values(:,3);
plot(x,y,'.-','DisplayName','UWB_UKF_Rawd_CT_3D_All', 'MarkerSize',8, 'MarkerIndices', floor(marker_interval*2/3)+1:marker_interval:length(fitted_x3))

x = UWB_EKF_Rawd_CT_3D.signals.values(:,1);
y = UWB_EKF_Rawd_CT_3D.signals.values(:,3);
plot(x(enableKF),y(enableKF),'.-','DisplayName','UWB\_EKF\_Rawd\_CT\_3D\_Eliminated', 'MarkerSize',8, 'MarkerIndices', floor(marker_interval*2/3)+1:marker_interval:length(fitted_x3))

x = UWB_UKF_Rawd_CT_3D.signals.values(:,1);
y = UWB_UKF_Rawd_CT_3D.signals.values(:,3);
plot(x(enableKF),y(enableKF),'.-','DisplayName','UWB\_UKF\_Rawd\_CT\_3D\_Eliminated', 'MarkerSize',8, 'MarkerIndices', floor(marker_interval*2/3)+1:marker_interval:length(fitted_x3))

x = UWB_EKF_Rawd_CT.signals.values(:,1);
y = UWB_EKF_Rawd_CT.signals.values(:,3);
plot(x(enableKF),y(enableKF),'.-','DisplayName','UWB_EKF_Rawd_CT', 'MarkerSize',8, 'MarkerIndices', floor(marker_interval*2/3)+1:marker_interval:length(fitted_x3))

x = UWB_UKF_Rawd_CT.signals.values(:,1);
y = UWB_UKF_Rawd_CT.signals.values(:,3);
plot(x(enableKF),y(enableKF),'.-','DisplayName','UWB_UKF_Rawd_CT', 'MarkerSize',8, 'MarkerIndices', floor(marker_interval*2/3)+1:marker_interval:length(fitted_x3))

hold off;

% 定义marker_interval
marker_interval = 800; % 每隔800个点显示一个标记

% 绘制第一个子图：原始数据
figure
subplot(4, 1, 1);
x = uwb_raw_global_lidar(:,1);
y = uwb_raw_global_lidar(:,2);
plot(x, y, '-','LineWidth',0.001, 'DisplayName', 'UWB Raw')
hold on;
scatter(x(~valid_raw), y(~valid_raw), 32,'DisplayName', 'Non-triangular')
xlabel('x (m)');
ylabel('y (m)');
legend('NumColumns', 3, 'Location', 'north'); 
% title('Raw Data and Eliminated Points')
axis equal;
xlim([0,227])
ylim([-28,35.5])
% axis tight;
text(max(xlim)-3, max(ylim)-0.2, '(a)', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'right');

enableKF = Enable_KF.signals.values;
scatter(x(~enableKF), y(~enableKF), 16,'d', 'DisplayName', 'Eliminated')

x = UWB_EKF_Rawd_CT_3D.signals.values(:,1);
y = UWB_EKF_Rawd_CT_3D.signals.values(:,3);
plot(x(enableKF),y(enableKF),'^-.','DisplayName','EKF 3D LUGOT (\bf{ours}\rm)', 'MarkerSize',4, 'MarkerIndices', 1:marker_interval:length(fitted_x3))

x = UWB_UKF_Rawd_CT_3D.signals.values(:,1);
y = UWB_UKF_Rawd_CT_3D.signals.values(:,3);
plot(x(enableKF),y(enableKF),'.:','DisplayName','UKF 3D LUGOT (\bf{ours}\rm)', 'MarkerSize',8, 'MarkerIndices', floor(marker_interval*1/2)+1:marker_interval:length(fitted_x3))

hold off;

% 绘制第二个子图：UWB_EKF_Rawd_CT_3D
subplot(4, 1, 2);
x = uwb_global_lidar(:,1);
y = uwb_global_lidar(:,2);
plot(x, y, 's-','LineWidth',0.001, 'DisplayName', 'KF \itd_{\rm1}, \itd_{\rm2}', 'MarkerSize', 4, 'MarkerIndices', 1:marker_interval:length(fitted_x1))
hold on;

x = UWB_EKF_Rawd_CT_3D.signals.values(:,1);
y = UWB_EKF_Rawd_CT_3D.signals.values(:,3);
plot(x(enableKF), y(enableKF), '^-.', 'DisplayName', 'EKF 3D LUGOT (\bf{ours}\rm)', 'MarkerSize', 4, 'MarkerIndices', floor(marker_interval*1/3)+1:marker_interval:length(fitted_x3))
x = UWB_UKF_Rawd_CT_3D.signals.values(:,1);
y = UWB_UKF_Rawd_CT_3D.signals.values(:,3);
plot(x(enableKF),y(enableKF),'.:','DisplayName','UKF 3D LUGOT (\bf{ours}\rm)', 'MarkerSize',8, 'MarkerIndices', floor(marker_interval*2/3)+1:marker_interval:length(fitted_x3))

xlabel('x (m)');
ylabel('y (m)');
legend('NumColumns', 3, 'Location', 'north'); 
% title('UWB_EKF_Rawd_CT_3D')
axis equal;
xlim([0,227])
ylim([-28,35.5])
% axis tight;
text(max(xlim)-3, max(ylim)-0.2, '(b)', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'right');
hold off;

% 绘制第三个子图：UWB_UKF_Rawd_CT_3D
subplot(4, 1, 3);
x = UWB_UKF_Rawd_CT_3D_All.signals.values(:,1);
y = UWB_UKF_Rawd_CT_3D_All.signals.values(:,3);
plot(x,y,'x-','DisplayName','UKF 3D All', 'MarkerSize',4, 'MarkerIndices', 1:marker_interval:length(fitted_x3))
hold on;
x = UWB_UKF_Rawd_CT_3D_All.signals.values(:,1);
y = UWB_UKF_Rawd_CT_3D_All.signals.values(:,3);
plot(x, y, '+--', 'DisplayName', 'UKF 3D All', 'MarkerSize', 4, 'MarkerIndices', floor(marker_interval*1/4)+1:marker_interval:length(fitted_x3))

x = UWB_EKF_Rawd_CT_3D.signals.values(:,1);
y = UWB_EKF_Rawd_CT_3D.signals.values(:,3);
plot(x(enableKF), y(enableKF), '^-.', 'DisplayName', 'EKF 3D LUGOT (\bf{ours}\rm)', 'MarkerSize', 4, 'MarkerIndices', floor(marker_interval*2/4)+1:marker_interval:length(fitted_x3))
x = UWB_UKF_Rawd_CT_3D.signals.values(:,1);
y = UWB_UKF_Rawd_CT_3D.signals.values(:,3);
plot(x(enableKF), y(enableKF), '.:', 'DisplayName', 'UKF 3D LUGOT (\bf{ours}\rm)', 'MarkerSize', 8, 'MarkerIndices', floor(marker_interval*3/4)+1:marker_interval:length(fitted_x3))

xlabel('x (m)');
ylabel('y (m)');
legend('NumColumns', 2, 'Location', 'northwest'); 
% title('UWB_UKF_Rawd_CT_3D')
axis equal;
xlim([0,227])
ylim([-28,35.5])
% axis tight;
text(max(xlim)-3, max(ylim)-0.2, '(c)', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'right');
hold off;

% 绘制第三个子图：UWB_UKF_Rawd_CT_3D
subplot(4, 1, 4);
x = UWB_EKF_Rawd_CT.signals.values(:,1);
y = UWB_EKF_Rawd_CT.signals.values(:,3);
plot(x(enableKF),y(enableKF),'x-','DisplayName','EKF 2D LUGOT', 'MarkerSize',4, 'MarkerIndices', 1:marker_interval:length(fitted_x3))
hold on;
x = UWB_UKF_Rawd_CT.signals.values(:,1);
y = UWB_UKF_Rawd_CT.signals.values(:,3);
plot(x(enableKF),y(enableKF),'+--','DisplayName','UKF 2D LUGOT', 'MarkerSize',4, 'MarkerIndices', floor(marker_interval*1/4)+1:marker_interval:length(fitted_x3))

x = UWB_EKF_Rawd_CT_3D.signals.values(:,1);
y = UWB_EKF_Rawd_CT_3D.signals.values(:,3);
plot(x(enableKF), y(enableKF), '^-.', 'DisplayName', 'EKF 3D LUGOT (\bf{ours}\rm)', 'MarkerSize', 4, 'MarkerIndices', floor(marker_interval*2/4)+1:marker_interval:length(fitted_x3))
x = UWB_UKF_Rawd_CT_3D.signals.values(:,1);
y = UWB_UKF_Rawd_CT_3D.signals.values(:,3);
plot(x(enableKF), y(enableKF), '.:', 'DisplayName', 'UKF 3D LUGOT (\bf{ours}\rm)', 'MarkerSize', 8, 'MarkerIndices', floor(marker_interval*3/4)+1:marker_interval:length(fitted_x3))

xlabel('x (m)');
ylabel('y (m)');
legend('NumColumns', 2, 'Location', 'northwest'); 
% title('UWB_UKF_Rawd_CT_3D')
axis equal;
xlim([0,227])
ylim([-28,35.5])
% axis tight;
text(max(xlim)-3, max(ylim)-0.2, '(d)', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'right');
hold off;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure
res1 = UWB_EKF_Rawd_CT_3D_Residual.signals.values(:,1);
res2 = UWB_EKF_Rawd_CT_3D_Residual.signals.values(:,2);
subplot(2,2,1)
plot(time_uwb,res1,'.-','DisplayName','UWB\_UKF\_Rawd\_CT\_3D\_d1\_Residual')
hold on
scatter(time_uwb(~enableKF),res1(~enableKF), 'd','DisplayName','Eliminated')
xlabel('Time (s)');
ylabel('Residual (m)');
legend show

ax2 = subplot(2,2,2);
plot(time_uwb,res1,'DisplayName','UWB\_UKF\_Rawd\_CT\_3D\_d1\_Residual')
hold on
scatter(time_uwb(~enableKF),res1(~enableKF), 'd','DisplayName','Eliminated')
xlabel('Time (s)')
ylabel('Residual (m)')
ylim(ax2,[-0.2,0.2])
legend show

subplot(2,2,3)
plot(time_uwb,res2,'.-','DisplayName','UWB\_UKF\_Rawd\_CT\_3D\_d2\_Residual')
hold on
scatter(time_uwb(~enableKF),res2(~enableKF), 'd','DisplayName','Eliminated')
xlabel('Time (s)');
ylabel('Residual (m)');
legend show

ax4 = subplot(2,2,4);
plot(time_uwb,res2,'DisplayName','UWB\_UKF\_Rawd\_CT\_3D\_d2\_Residual')
hold on
scatter(time_uwb(~enableKF),res2(~enableKF), 'd','DisplayName','Eliminated')
xlabel('Time (s)')
ylabel('Residual (m)')
ylim(ax4,[-0.2,0.2])
legend show
% plot(time_uwb,UWB_EKF_Rawd_CT_3D_Residual.signals.values(:,2),'.-','DisplayName','UWB\_UKF\_Rawd\_CT\_3D\_d2\_Residual')

hold off

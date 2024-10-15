% 使用smoothing spline拟合UWB轨迹
% 使用 unique 函数找到唯一时间值及其索引
[unique_time_uwb, idx_unique] = unique(time_uwb, 'stable');

uwb_raw_global_lidar_x = uwb_raw_global_lidar(:,1);
uwb_raw_global_lidar_y = uwb_raw_global_lidar(:,2);
% 使用这些索引来筛选出对应的 x 和 y 数值
% unique_x = uwb_raw_global_lidar_x(idx_unique);
% unique_y = uwb_raw_global_lidar_y(idx_unique);

time_uwb_valid = time_uwb(valid_raw);
uwb_raw_global_lidar_x_valid = uwb_raw_global_lidar_x(valid_raw);
uwb_raw_global_lidar_y_valid = uwb_raw_global_lidar_y(valid_raw);

criterion_valid_ = criterion_valid.signals.values;
time_uwb_and_valid = time_uwb(valid_raw & criterion_valid_);
uwb_raw_global_lidar_x_and_valid = uwb_raw_global_lidar_x(valid_raw & criterion_valid_);
uwb_raw_global_lidar_y_and_valid = uwb_raw_global_lidar_y(valid_raw & criterion_valid_);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[fitted_x1, fitted_y1, sp_x, sp_y] = fitSmoothingSpline(time_uwb, uwb_raw_global_lidar_x, uwb_raw_global_lidar_y, time_uwb_and_valid);
% computeAndPlotResiduals(time_uwb, uwb_raw_global_lidar_x, fitted_x, uwb_raw_global_lidar_y, fitted_y);
[rmse_x1, rmse_y1, rmse_dis1] = computeAndPlotResiduals(time_uwb_and_valid, uwb_raw_global_lidar_x_and_valid, fitted_x1, uwb_raw_global_lidar_y_and_valid, fitted_y1);
[~, ~, ~, ~, integral_cumtrapz1, integral_trapz1] = computeVelocityAndAngle(unique_time_uwb, sp_x, sp_y);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[fitted_x2, fitted_y2, sp_x, sp_y] = fitSmoothingSpline(time_uwb_valid, uwb_raw_global_lidar_x_valid, uwb_raw_global_lidar_y_valid, time_uwb_and_valid);
% computeAndPlotResiduals(time_uwb, uwb_raw_global_lidar_x, fitted_x, uwb_raw_global_lidar_y, fitted_y);
[rmse_x2, rmse_y2, rmse_dis2] = computeAndPlotResiduals(time_uwb_and_valid, uwb_raw_global_lidar_x_and_valid, fitted_x2, uwb_raw_global_lidar_y_and_valid, fitted_y2);
[~, ~, ~, ~, integral_cumtrapz2, integral_trapz2] = computeVelocityAndAngle(unique_time_uwb, sp_x, sp_y);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 调用函数进行平滑样条拟合
[fitted_x3, fitted_y3, sp_x, sp_y] = fitSmoothingSpline(time_uwb_and_valid, uwb_raw_global_lidar_x_and_valid, uwb_raw_global_lidar_y_and_valid, time_uwb_and_valid);
% 调用函数计算残差并计算RMSE
[rmse_x3, rmse_y3, rmse_dis3] = computeAndPlotResiduals(time_uwb_and_valid, uwb_raw_global_lidar_x_and_valid, fitted_x3, uwb_raw_global_lidar_y_and_valid, fitted_y3);
% 调用函数计算速度和角度
[ax, ay, a_scalar, alpha, integral_cumtrapz3, integral_trapz3] = computeVelocityAndAngle(unique_time_uwb, sp_x, sp_y);

% 绘制三组数据的拟合结果
figure
x = uwb_raw_global_lidar(:,1);
y = uwb_raw_global_lidar(:,2);
plot(x, y,'LineWidth',0.001,'DisplayName','Raw UWB');
hold on;
scatter(x(~valid_raw),y(~valid_raw), 32,'DisplayName','Non-triangular')
axis equal; % 设置x轴和y轴的比例尺相同
axis tight; % 调整坐标轴的范围以紧凑地显示图形
xlabel('x (m)');
ylabel('y (m)');
scatter(x(~criterion_valid_),y(~criterion_valid_), 16,'d','DisplayName','Criterion Invalid')
marker_interval = 800; % 每隔800个点显示一个标记
% 添加标记，但不影响整条曲线显示
hold on;
plot1 = plot(fitted_x1, fitted_y1, 's--', 'DisplayName', 'Fit All', 'MarkerSize',4, 'MarkerIndices', 1:marker_interval:length(fitted_x1));
plot2 = plot(fitted_x2, fitted_y2, '^-.', 'DisplayName', 'Fit \\ Non-triangular', 'MarkerSize',4, 'MarkerIndices', floor(marker_interval/3)+1:marker_interval:length(fitted_x2));
plot3 = plot(fitted_x3, fitted_y3, '.:', 'DisplayName', 'Fit \\ Criterion Invalid', 'MarkerSize',8, 'MarkerIndices', floor(marker_interval*2/3)+1:marker_interval:length(fitted_x3));
plot1.Color(4) = 0.8;
plot2.Color(4) = 0.8;
plot3.Color(4) = 0.8;
% legend show;
% legend('Orientation','horizontal')
lgd = legend('show');
lgd.NumColumns = 3; % 设置legend显示为两列
legend('Location','northoutside') 
hold off

% 绘制柱状图
figure;
% 绘制RMSE柱状图
subplot(1, 2, 1);
hBar = bar([rmse_x1, rmse_y1, rmse_dis1; rmse_x2, rmse_y2, rmse_dis2; rmse_x3, rmse_y3, rmse_dis3]');
set(gca, 'xticklabel', {'RMSE_{\itx}', 'RMSE_{\ity}', 'RMSE_{\itd}'});
% xlabel('Metrics');
ylabel('RMSE Value (m)');
legend({'Raw UWB', 'Non-triangular', 'Criterion Invalid'},'Location','west');
title('(b) Comparison of RMSE');
ax1 = gca;
grid on;
ax1.XGrid = 'off';
ax1.YGrid = 'on';
% 添加数值标签
for i = 1:length(hBar)
    xdata = hBar(i).XData + hBar(i).XOffset; % 获取x坐标
    ydata = hBar(i).YData; % 获取y坐标
    for j = 1:length(xdata)
        text(xdata(j), ydata(j), num2str(ydata(j), '%.4f'), 'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle', 'Rotation', 75);
    end
end

% 绘制Integral of α²柱状图
subplot(1, 2, 2);
% vals = [1 3 5; 3 2 7; 3 4 2];
hBar = bar([integral_trapz1, integral_trapz2, integral_trapz3],'FaceColor','flat');
% 设置颜色
colors = [
    0 114 189;  % 颜色1
    217 83 25;  % 颜色2
    237 177 32;  % 颜色3
]/255.;
% 将颜色应用到每个柱子上
for k = 1:3%size(vals, 2)
    hBar.CData(k,:) = colors(k,:);
end
set(gca, 'xticklabel', {'Raw UWB','Non-triangular', 'Criterion Invalid'});
% xlabel('Metrics');
% ylabel('Integral of \omega^2');
% ylabel('Integral of $\dot{\psi}^2 \, (\mathrm{(rad/s)^2})$', 'Interpreter', 'latex');
ylabel('Integral of Angular Acceleration Square (rad^2/s^3)');
% ylabel('$\int{{\dot{\psi}}^2dt}$', 'Interpreter', 'latex');
title('(c)'); %Comparison of Integral of Turn Rate Square
ax1 = gca;
grid on;
ax1.XGrid = 'off';
ax1.YGrid = 'on';
% 添加数值标签
for i = 1:length(hBar)
    xdata = hBar(i).XData + hBar(i).XOffset; % 获取x坐标
    ydata = hBar(i).YData; % 获取y坐标
    for j = 1:length(xdata)
        text(xdata(j), ydata(j), num2str(ydata(j), '%.2f'), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom'); % , 'Rotation', -75
    end
end
% 添加旋转文字
text(max(xlim)-0.2, min(ylim)+0.3, '\itInitial data (indexes∈[1,3]) are not considered', 'Rotation', 90, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');

clear x y

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if numel(dbstack) <= 1
%     [final_p_x, final_p_y] = interactive_smoothing_spline(time_uwb, uwb_raw_global_lidar_x, uwb_raw_global_lidar_y)
%     [final_p_x, final_p_y] = interactive_smoothing_spline(time_uwb_valid, uwb_raw_global_lidar_x_valid, uwb_raw_global_lidar_y_valid)
%     [final_p_x, final_p_y] = interactive_smoothing_spline(time_uwb_and_valid, uwb_raw_global_lidar_x_and_valid, uwb_raw_global_lidar_y_and_valid)
end

function [fitted_x, fitted_y, sp_x, sp_y] = fitSmoothingSpline(time_in, x_in, y_in, time_out)
    % 为 uwb_raw_global_lidar_x 设置的平滑参数
    p_x = 0.2747; % 0.0724;
    % 使用 csaps 函数进行平滑样条拟合
    sp_x = csaps(time_in, x_in, p_x);
    % 在原有的 time_uwb 上评估这个样条曲线，得到拟合后的 x 值
    fitted_x = fnval(sp_x, time_out);

    % 为 uwb_raw_global_lidar_y 设置的平滑参数
    p_y = 0.0296; % 0.0132;
    % 使用 csaps 函数进行平滑样条拟合
    sp_y = csaps(time_in, y_in, p_y);
    % 在原有的 time_uwb 上评估这个样条曲线，得到拟合后的 y 值
    fitted_y = fnval(sp_y, time_out);
end

function [rmse_x,rmse_y,rmse_dis] = computeAndPlotResiduals(time_uwb_and_valid, uwb_raw_global_lidar_x, fitted_x, uwb_raw_global_lidar_y, fitted_y)
    % 计算Residual
    figure;
    subplot(3,2,1);
    plot(time_uwb_and_valid, uwb_raw_global_lidar_x - fitted_x, 'DisplayName', 'smoothing spline and valid x');
    hold on;
    xlabel('Time (s)');
    ylabel('Residual (m)');
    title('smoothing spline and valid x');

    ax2 = subplot(3,2,2);
    plot(time_uwb_and_valid, uwb_raw_global_lidar_x - fitted_x, 'DisplayName', 'smoothing spline and valid x');
    xlabel('Time (s)');
    ylabel('Residual (m)');
    ylim(ax2, [-1.5, 1.5]);
    title('smoothing spline and valid x');

    subplot(3,2,3);
    plot(time_uwb_and_valid, uwb_raw_global_lidar_y - fitted_y, 'DisplayName', 'smoothing spline and valid y');
    xlabel('Time (s)');
    ylabel('Residual (m)');
    title('smoothing spline and valid y');

    ax4 = subplot(3,2,4);
    plot(time_uwb_and_valid, uwb_raw_global_lidar_y - fitted_y, 'DisplayName', 'smoothing spline and valid y');
    xlabel('Time (s)');
    ylabel('Residual (m)');
    ylim(ax4, [-2, 2]);
    title('smoothing spline and valid y');

    subplot(3,2,5);
    plot(time_uwb_and_valid, sqrt((uwb_raw_global_lidar_y - fitted_y).^2 + (uwb_raw_global_lidar_x - fitted_x).^2), 'DisplayName', 'smoothing spline and valid distance');
    xlabel('Time (s)');
    ylabel('Residual (m)');
    title('smoothing spline and valid distance');

    ax4 = subplot(3,2,6);
    plot(time_uwb_and_valid, sqrt((uwb_raw_global_lidar_y - fitted_y).^2 + (uwb_raw_global_lidar_x - fitted_x).^2), 'DisplayName', 'smoothing spline and valid distance');
    xlabel('Time (s)');
    ylabel('Residual (m)');
    title('smoothing spline and valid distance');
    ylim(ax4, [-1, 4]);
    hold off;

    % 计算并输出均方根误差 (RMSE)
    rmse_x = sqrt(mean((uwb_raw_global_lidar_x - fitted_x).^2));
%     fprintf('The RMSE for x coordinates is: %f\n', rmse_x);
    rmse_y = sqrt(mean((uwb_raw_global_lidar_y - fitted_y).^2));
%     fprintf('The RMSE for y coordinates is: %f\n', rmse_y);
    differences = [uwb_raw_global_lidar_x,uwb_raw_global_lidar_y] - [fitted_x, fitted_y];
    distances = sqrt(sum(differences.^2, 2));
    rmse_dis = sqrt(mean(distances.^2));
%     fprintf('The RMSE for 2-D positioning is: %f\n', rmse_dis);
end

function [ax, ay, a, alpha, integral_cumtrapz, integral_trapz] = computeVelocityAndAngle(unique_time_uwb, sp_x, sp_y)
    % 计算速度和角度
    fitted_x_unique = fnval(sp_x, unique_time_uwb);
    fitted_y_unique = fnval(sp_y, unique_time_uwb);
    
    % 计算速度
    sp_vx = fnder(sp_x); 
    sp_vy = fnder(sp_y); 
    vx = fnval(sp_vx, unique_time_uwb);
    vy = fnval(sp_vy, unique_time_uwb);
    v = sqrt(vx.^2 + vy.^2);
    
    % 计算角度
    psi = atan2(vy, vx); 

    for i = 2:length(psi)
        dtheta = psi(i) - psi(i-1);
        if dtheta > pi
            psi(i:end) = psi(i:end) - 2 * pi; 
        elseif dtheta < -pi
            psi(i:end) = psi(i:end) + 2 * pi; 
        end
    end

    % 计算加速度
    sp_ax = fnder(sp_x,2); 
    sp_ay = fnder(sp_y,2); 
%     sp_ax = fnder(sp_vx); 
%     sp_ay = fnder(sp_vy);
    ax = fnval(sp_ax, unique_time_uwb);
    ay = fnval(sp_ay, unique_time_uwb);
%     a = sqrt(ax.^2 + ay.^2);
    a = (vx .* ax + vy .* ay) ./ v;

    % 计算角速度
    omega = (vx .* ay - vy .* ax) ./ (vx.^2 + vy.^2);
    % 计算角加速度
    numerator = (ay .* vx - ax .* vy) .* (vx.^2 + vy.^2) - (vy .* ax - vx .* ay) .* 2 .* (vx .* ax + vy .* ay);
    denominator = (vx.^2 + vy.^2).^2;
    alpha = numerator ./ denominator;

    % 绘制速度、角度、角速度等随时间变化的图
    figure;
    
    subplot(3,1,1);
    plot(unique_time_uwb, v * 3.6);
    xlabel('Time (s)');
    ylabel('Velocity (km/h)');
    title('Velocity over Time');

    subplot(3,1,2);
    plot(unique_time_uwb, psi * 180.0 / pi);
    xlabel('Time (s)');
    ylabel('$\psi\,(\mathrm{^\circ})$', 'Interpreter', 'latex');
    title('Continue Heading Angle over Time');

    ax3 = subplot(3,1,3);
    plot(unique_time_uwb, omega * 180.0 / pi);
    xlabel('Time (s)');
    ylabel('$\dot{\psi}\,(\mathrm{^\circ/s})$', 'Interpreter', 'latex');
%     ylim(ax3, [-200, 100]);
    title('Turn Rate over Time');

    % 绘制加速度图
    figure;
    ax1 = subplot(4,1,1);
    plot(unique_time_uwb, ax);
    xlabel('Time (s)');
    ylabel('\ita_x\rm (m/s^2)');
%     ylim(ax1, [-4, 7]);
    title('x Linear Acceleration over Time');

    ax2 = subplot(4,1,2);
    plot(unique_time_uwb, ay);
    xlabel('Time (s)');
    ylabel('\ita_y\rm (m/s^2)');
%     ylim(ax2, [-3, 3]);
    title('y Linear Acceleration over Time');

    ax3 = subplot(4,1,3);
    plot(unique_time_uwb, a);
    xlabel('Time (s)');
    ylabel('\ita\rm (m/s^2)');
%     ylim(ax3, [-4, 4]);
    title('Total Linear Acceleration over Time');
    
    ax4 = subplot(4,1,4);
    plot(unique_time_uwb, alpha * 180.0 / pi);
    xlabel('Time (s)');
    ylabel('$\ddot{\psi}\,(\mathrm{^\circ/s^2})$', 'Interpreter', 'latex');
%     ylim(ax4, [-40000, 30000]);
    title('Angular Acceleration over Time');
    
    alpha_squared = alpha .^ 2; 
    %去除开始不稳定阶段的数据
    integral_cumtrapz = cumtrapz(unique_time_uwb(3+1:end), alpha_squared(3+1:end)); % cumtrapz(unique_time_uwb, alpha_squared); % cumtrapz(unique_time_uwb(227:end), alpha_squared(227:end))
    integral_trapz = trapz(unique_time_uwb(3+1:end), alpha_squared(3+1:end)); % trapz(unique_time_uwb, alpha_squared); % trapz(unique_time_uwb(227:end), alpha_squared(227:end))
    disp('使用 trapz 进行的总积分结果:');
    disp(integral_trapz);
end


function [final_p_x, final_p_y] = interactive_smoothing_spline(time_uwb, uwb_raw_global_lidar_x, uwb_raw_global_lidar_y)
    % 初始化参数
    p_x = 0.0181;
    p_y = 0.0263;

    % 创建图形界面
    f = figure('Name', 'Interactive Smoothing Spline', 'Position', [100 100 900 800], ...
               'CloseRequestFcn', @closeRequest);

    % 创建三个子图
    ax1 = axes('Parent', f, 'Position', [0.1 0.66 0.8 0.3]);
    ax3 = axes('Parent', f, 'Position', [0.1 0.33 0.8 0.3]);
    ax2 = axes('Parent', f, 'Position', [0.1 0.01 0.8 0.3]);

    % 添加调整 p_x 的滑动条
    slider_px = uicontrol('Style', 'slider', 'Min', 0, 'Max', 1, 'Value', p_x, 'SliderStep', [0.0001 0.1], ...
                          'Position', [150 770 700 20], 'Callback', @updatePlot);
    addlistener(slider_px, 'Value', 'PostSet', @updatePlot);

    % 添加调整 p_y 的滑动条
    slider_py = uicontrol('Style', 'slider', 'Min', 0, 'Max', 1, 'Value', p_y, 'SliderStep', [0.0001 0.1], ...
                          'Position', [150 740 700 20], 'Callback', @updatePlot);
    addlistener(slider_py, 'Value', 'PostSet', @updatePlot);

    % 添加可编辑文本框以显示并修改 p_x 和 p_y 的值
    edit_px = uicontrol('Style', 'edit', 'Position', [860 770 100 20], 'String', sprintf('%.4f', p_x), ...
                        'Callback', @editPx);
    edit_py = uicontrol('Style', 'edit', 'Position', [860 740 100 20], 'String', sprintf('%.4f', p_y), ...
                        'Callback', @editPy);

    % 添加标签
    uicontrol('Style', 'text', 'Position', [50 770 100 20], 'String', 'Smoothing p_x:');
    uicontrol('Style', 'text', 'Position', [50 740 100 20], 'String', 'Smoothing p_y:');

    % 初始绘图
    plot(ax1, time_uwb, uwb_raw_global_lidar_x, 'DisplayName', 'Raw X');
    plot(ax3, time_uwb, uwb_raw_global_lidar_y, 'DisplayName', 'Raw Y');
    hold(ax1, 'on');
    hold(ax3, 'on');
    plot(ax2, uwb_raw_global_lidar_x, uwb_raw_global_lidar_y, '.-', 'DisplayName', 'Raw XY');
    hold(ax2, 'on');
    axis(ax2, 'equal');
    legend(ax1, 'show');
    legend(ax3, 'show');
    legend(ax2, 'show');

    % 从文本框读取并更新 p_x
    function editPx(source, ~)
        new_px = str2double(get(source, 'String'));
        set(slider_px, 'Value', new_px);
        updatePlot();
    end

    % 从文本框读取并更新 p_y
    function editPy(source, ~)
        new_py = str2double(get(source, 'String'));
        set(slider_py, 'Value', new_py);
        updatePlot();
    end

    % 更新图形的函数
    function updatePlot(~, ~)
        % 从滑动条读取最新的 p_x 和 p_y 值
        p_x = get(slider_px, 'Value');
        p_y = get(slider_py, 'Value');

        % 更新文本框
        set(edit_px, 'String', sprintf('%.4f', p_x));
        set(edit_py, 'String', sprintf('%.4f', p_y));

        % 计算新的平滑样条
        sp_x = csaps(time_uwb, uwb_raw_global_lidar_x, p_x);
        sp_y = csaps(time_uwb, uwb_raw_global_lidar_y, p_y);

        % 计算拟合值
        fitted_x = fnval(sp_x, time_uwb);
        fitted_y = fnval(sp_y, time_uwb);

        % 更新图形
        if isvalid(ax1)
            cla(ax1);
            plot(ax1, time_uwb, uwb_raw_global_lidar_x, 'DisplayName', 'Raw X');
            plot(ax1, time_uwb, fitted_x, '.-', 'DisplayName', 'Fitted X');
            legend(ax1, 'show');
        end
        if isvalid(ax3)
            cla(ax3);
            plot(ax3, time_uwb, uwb_raw_global_lidar_y, 'DisplayName', 'Raw Y');
            plot(ax3, time_uwb, fitted_y, '.-', 'DisplayName', 'Fitted Y');
            legend(ax3, 'show');
        end
        if isvalid(ax2)
            cla(ax2);
            plot(ax2, uwb_raw_global_lidar_x, uwb_raw_global_lidar_y, 'DisplayName', 'Raw XY');
            plot(ax2, fitted_x, fitted_y, '.-', 'DisplayName', 'Fitted XY');
            legend(ax2, 'show');
        end
    end

    % 窗口关闭时的处理函数
    function closeRequest(~, ~)
        final_p_x = get(slider_px, 'Value');
        final_p_y = get(slider_py, 'Value');
        delete(f);
    end

    % 等待图形界面关闭
    uiwait(f);
end
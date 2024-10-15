% 使用smoothing spline拟合UWB距离

% 使用 unique 函数找到唯一时间值及其索引
[unique_time_uwb, idx_unique] = unique(time_uwb, 'stable');

uwb_raw_global_lidar_x = uwb_raw_global_lidar(:,1);
uwb_raw_global_lidar_y = uwb_raw_global_lidar(:,2);

time_uwb_valid = time_uwb(valid_raw);

criterion_valid_ = criterion_valid.signals.values;
time_uwb_and_valid = time_uwb(valid_raw & criterion_valid_);
uwb_raw_global_lidar_x_and_valid = uwb_raw_global_lidar_x(valid_raw & criterion_valid_);
uwb_raw_global_lidar_y_and_valid = uwb_raw_global_lidar_y(valid_raw & criterion_valid_);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 调用函数进行平滑样条拟合
[fitted_x3, fitted_y3, sp_x, sp_y] = fitSmoothingSpline(time_uwb_and_valid, uwb_raw_global_lidar_x_and_valid, uwb_raw_global_lidar_y_and_valid, time_uwb_and_valid);
% 调用函数计算速度和角度
% [ax, ay, a_scalar, alpha, integral_cumtrapz3, integral_trapz3] = computeVelocityAndAngle(time_uwb_and_valid, sp_x, sp_y);

clear x y
% 拟合并计算拟合曲线的速度和加速度
distance_relative = vecnorm(rel_pos_raw(valid_raw & criterion_valid_), 2, 2); % 2 表示使用欧几里得范数，第二个 2 表示沿行计算
[fitted_d_rel, sp_d_rel] = fitSmoothingSpline1D(time_uwb_and_valid, distance_relative, time_uwb_and_valid);
[v_rel, a_rel] = computeVelocityAndAcc1D(time_uwb_and_valid, sp_d_rel);

% 绘制结果
figure;
plot(time_uwb_and_valid, distance_relative);
xlabel('Time (s)');
ylabel('Relative Distance (m)');
grid on;
% 计算 y 值的最小和最大值
y_min = min(distance_relative);
y_max = max(distance_relative);
% 在图中绘制 y 值的最小和最大取值范围
yline(y_min, '--r', ['Min Value: ' num2str(y_min)], 'LabelHorizontalAlignment', 'center');
yline(y_max, '--b', ['Max Value: ' num2str(y_max)], 'LabelHorizontalAlignment', 'center');
% % 显示最大值和最小值的位置
% text(mean(time_uwb_and_valid), y_min, ['Min: ' num2str(y_min)], 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center', 'Color', 'r');
% text(mean(time_uwb_and_valid), y_max, ['Max: ' num2str(y_max)], 'VerticalAlignment', 'top', 'HorizontalAlignment', 'center', 'Color', 'g');



residual = vecnorm([uwb_raw_global_lidar_x_and_valid - fitted_x3, uwb_raw_global_lidar_y_and_valid - fitted_y3], 2, 2);
plot_comparison(distance_relative, residual, 'Relative Distance', 'Relative Distance', 'm');
plot_comparison(v_rel, residual, 'Relative Velocity', 'Relative Velocity', 'm/s');
plot_comparison(a_rel, residual, 'Relative Acceleration', 'Relative Acceleration', 'm/s²');
polar_angle = atan2(y_raw, x_raw(:,1));
polar_angle_and_valid = polar_angle(valid_raw & criterion_valid_);
plot_comparison(polar_angle_and_valid, residual, 'Polar Angle', 'Polar Angle', 'rad');
% UWB_EKF_Rawd_CT_3D
residual = vecnorm([value3(:,1) - fitted_x3, value3(:,2) - fitted_y3], 2, 2);
plot_comparison(distance_relative, residual, 'Relative Distance', 'Relative Distance', 'm');
plot_comparison(v_rel, residual, 'Relative Velocity', 'Relative Velocity', 'm/s');
plot_comparison(a_rel, residual, 'Relative Acceleration', 'Relative Acceleration', 'm/s²');
polar_angle = atan2(y_raw, x_raw(:,1));
polar_angle_and_valid = polar_angle(valid_raw & criterion_valid_);
plot_comparison(polar_angle_and_valid, residual, 'Polar Angle', 'Polar Angle', 'rad');
% UWB_UKF_Rawd_CT_3D
residual = vecnorm([value4(:,1) - fitted_x3, value4(:,2) - fitted_y3], 2, 2);
plot_comparison(distance_relative, residual, 'Relative Distance', 'Relative Distance', 'm');
plot_comparison(v_rel, residual, 'Relative Velocity', 'Relative Velocity', 'm/s');
plot_comparison(a_rel, residual, 'Relative Acceleration', 'Relative Acceleration', 'm/s²');
polar_angle = atan2(y_raw, x_raw(:,1));
polar_angle_and_valid = polar_angle(valid_raw & criterion_valid_);
plot_comparison(polar_angle_and_valid, residual, 'Polar Angle', 'Polar Angle', 'rad');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if numel(dbstack) <= 1
%     final_p_d = interactive_smoothing_spline1D(time_uwb_and_valid, distance_relative);
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

function [fitted_x, sp_x] = fitSmoothingSpline1D(time_in, x_in, time_out)
    % 设置的平滑参数
    p_x = 0.2827846224721795; % 0.2747
    % 使用 csaps 函数进行平滑样条拟合
    sp_x = csaps(time_in, x_in, p_x);
    % 在原有的 time_uwb 上评估这个样条曲线，得到拟合后的 x 值
    fitted_x = fnval(sp_x, time_out);
end

function [ax, ay, a, alpha, integral_cumtrapz, integral_trapz] = computeVelocityAndAngle(unique_time_uwb, sp_x, sp_y)
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

function [vx, ax] = computeVelocityAndAcc1D(unique_time, sp_x)   
    % 计算速度
    sp_vx = fnder(sp_x); 
    vx = fnval(sp_vx, unique_time);

    % 计算加速度
    sp_ax = fnder(sp_x,2); 
    ax = fnval(sp_ax, unique_time);

   % 绘制速度、角度、角速度等随时间变化的图
    figure;
    subplot(2,1,1);
    plot(unique_time, vx * 3.6);
    xlabel('Time (s)');
    ylabel('Relative Velocity (km/h)');
    title('Relative Velocity over Time');

    % 绘制加速度图
    subplot(2,1,2);
    plot(unique_time, ax);
    xlabel('Time (s)');
    ylabel('\ita_x\rm (m/s^2)');
%     ylim(ax1, [-4, 7]);
    title('d Linear Acceleration over Time');
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

function [final_p_x] = interactive_smoothing_spline1D(time, x)
    % 初始化参数
    p_x = 0.2827846224721795; % 0.0181;

    % 创建图形界面
    f = figure('Name', 'Interactive Smoothing Spline', 'Position', [100 100 900 800], ...
               'CloseRequestFcn', @closeRequest);
    % 创建三个子图
    ax1 = axes('Parent', f, 'Position', [0.1 0.66 0.8 0.3]);
    % 添加调整 p_x 的滑动条
    slider_px = uicontrol('Style', 'slider', 'Min', 0, 'Max', 1, 'Value', p_x, 'SliderStep', [0.0001 0.1], ...
                          'Position', [150 770 700 20], 'Callback', @updatePlot);
    addlistener(slider_px, 'Value', 'PostSet', @updatePlot);
    % 添加可编辑文本框以显示并修改 p_x 的值
    edit_px = uicontrol('Style', 'edit', 'Position', [860 770 100 20], 'String', sprintf('%.4f', p_x), ...
                        'Callback', @editPx);
    % 添加标签
    uicontrol('Style', 'text', 'Position', [50 770 100 20], 'String', 'Smoothing p_x:');
    % 初始绘图
    plot(ax1, time, x, 'DisplayName', 'Raw X');
    hold(ax1, 'on');
    legend(ax1, 'show');

    % 从文本框读取并更新 p_x
    function editPx(source, ~)
        new_px = str2double(get(source, 'String'));
        set(slider_px, 'Value', new_px);
        updatePlot();
    end

    % 更新图形的函数
    function updatePlot(~, ~)
        % 从滑动条读取最新的 p_x 值
        p_x = get(slider_px, 'Value');
        % 更新文本框
        set(edit_px, 'String', sprintf('%.4f', p_x));
        % 计算新的平滑样条
        sp_x = csaps(time, x, p_x);
        % 计算拟合值
        fitted_x = fnval(sp_x, time);

        % 更新图形
        if isvalid(ax1)
            cla(ax1);
            plot(ax1, time, x, 'DisplayName', 'Raw X');
            plot(ax1, time, fitted_x, '.-', 'DisplayName', 'Fitted X');
            legend(ax1, 'show');
        end
    end

    % 窗口关闭时的处理函数
    function closeRequest(~, ~)
        final_p_x = get(slider_px, 'Value');
        delete(f);
    end

    % 等待图形界面关闭
    uiwait(f);
end

function y_filtered = sliding_window_mean(x, y, window_length, mode)
% sliding_window_mean 基于 x 轴长度或分段数量的滑窗均值滤波
%
% 参数：
% x - 原始 x 轴数据
% y - 原始 y 轴数据
% window_length - 滑窗长度（对于 'length' 模式是 x 轴长度；对于 'segments' 模式是分段数量）
% mode - 滑窗长度模式，'length' 表示基于 x 轴长度，'segments' 表示基于指定分段数量

    % 判断模式
    if strcmp(mode, 'length')
        % 基于固定 x 轴长度的滑窗
        y_filtered = length_based_filter(x, y, window_length);
    elseif strcmp(mode, 'segments')
        % 基于分段数量的滑窗
        segment_length = (max(x) - min(x)) / window_length; % 每段的 x 轴长度
        y_filtered = length_based_filter(x, y, segment_length);
    else
        error('无效的模式。模式应为 "length" 或 "segments"。');
    end
end

function y_filtered = length_based_filter(x, y, window_length)
% length_based_filter 基于固定 x 轴长度的滑动均值滤波

    % 初始化滤波后的数据
    y_filtered = zeros(size(y));

    % 滑动窗口均值滤波
    for i = 1:length(x)
        % 确定当前窗口的起点和终点
        window_start = x(i) - window_length / 2;
        window_end = x(i) + window_length / 2;
        
        % 找到窗口范围内的索引
        window_idx = (x >= window_start) & (x <= window_end);
        
        % 计算该窗口范围内的均值
        y_filtered(i) = mean(y(window_idx));
    end
end

function [x_uniform, y_uniform_filtered] = uniform_resample(x, y_filtered, num_points)
% uniform_resample 对x轴上的滑窗结果进行均匀分布采样
%
% 参数：
% x - 原始 x 轴数据
% y_filtered - 滤波后的 y 值
% num_points - 希望输出的均匀分布的点数量
%
% 返回：
% x_uniform - 在 x 轴上均匀分布的点
% y_uniform_filtered - 对应的滤波后的 y 值

    % 生成 x 轴上均匀分布的点
    x_uniform = linspace(min(x), max(x), num_points);
    
    % 初始化存储均匀分布的 y 值
    y_uniform_filtered = zeros(1, num_points);
    
    % 对每个均匀分布的点，在原始的 x 轴上找到最近的点
    for i = 1:num_points
        % 找到最接近 x_uniform(i) 的原始 x 点的索引
        [~, idx] = min(abs(x - x_uniform(i)));
        
        % 将对应的 y 值存储
        y_uniform_filtered(i) = y_filtered(idx);
    end
end

function plot_comparison(v_or_a_rel, residual, title_prefix, x_label_prefix, x_unit)
% plot_comparison 用于绘制相对值和绝对值的对比图
%
% 参数：
% v_or_a_rel - 相对速度 (v_rel) 或相对加速度 (a_rel)
% residual - 拟合的相对距离与实际相对距离的残差
% title_prefix - 图形标题的前缀（如 "Relative Velocity" 或 "Relative Acceleration"）
% x_label_prefix - X轴标签的前缀（如 "Relative Velocity" 或 "Relative Acceleration"）
% x_unit - X轴变量的单位（如 "m/s" 或 "m/s²"）

    figure;

    % 第一个图 - 原始数据和滤波曲线
    subplot(2, 1, 1); % 两行一列的第一个子图
    sorted_v_res = sortrows([v_or_a_rel, abs(residual)], 1);
    hold on;
    scatter(sorted_v_res(:, 1), sorted_v_res(:, 2), 0.5, 'b.', 'DisplayName', 'Original Data');
    [sorted_v, sorted_res] = deal(sorted_v_res(:, 1), sorted_v_res(:, 2));
    y_filtered = sliding_window_mean(sorted_v, sorted_res, 20, 'segments');
    [sorted_v_uniform, y_uniform_filtered] = uniform_resample(sorted_v, y_filtered, 500);
    plot(sorted_v_uniform, y_uniform_filtered, 'r-', 'DisplayName', 'Filtered Data');
    title(['(a) ' title_prefix ' vs. Distance Residual']);
    xlabel([x_label_prefix ' (' x_unit ')']);
    ylabel('Distance Residual (m)');
    legend;
    grid on;
    box on; % 打开上下左右的坐标轴

    % 第二个图 - 绝对值处理后的数据和滤波曲线
    subplot(2, 1, 2); % 两行一列的第二个子图
    sorted_v_res_abs = sortrows([abs(v_or_a_rel), abs(residual)], 1);
    hold on;
    scatter(sorted_v_res_abs(:, 1), sorted_v_res_abs(:, 2), 0.5, [0.4660 0.6740 0.1880], '.', 'DisplayName', 'Original Data');
    [sorted_v_abs, sorted_res_abs] = deal(sorted_v_res_abs(:, 1), sorted_v_res_abs(:, 2));
    y_filtered_abs = sliding_window_mean(sorted_v_abs, sorted_res_abs, 20, 'segments');
    [sorted_v_abs_uniform, y_uniform_filtered_abs] = uniform_resample(sorted_v_abs, y_filtered_abs, 500);
    plot(sorted_v_abs_uniform, y_uniform_filtered_abs, 'm-', 'DisplayName', 'Filtered Data');
    title(['(b) ' 'Absolute ' title_prefix ' vs. Distance Residual']);
    xlabel(['Absolute ' x_label_prefix ' (' x_unit ')']);
    ylabel('Distance Residual (m)');
    legend;
    grid on;
    box on; % 打开上下左右的坐标轴

    % 调整图像布局，使其更加紧凑
    % sgtitle(['Comparison of Original and Filtered Data with ' title_prefix ' and Absolute Values']);
end

% 调用函数处理UWB_EKF_Rawd_CT_3D
analyze_signals(time_uwb, UWB_EKF_Rawd_CT_3D, valid_raw & criterion_valid_, idx_unique, ax, ay, a_scalar, alpha);

% 调用函数处理UWB_UKF_Rawd_CT_3D
analyze_signals(time_uwb, UWB_UKF_Rawd_CT_3D, valid_raw & criterion_valid_, idx_unique, ax, ay, a_scalar, alpha);

% 使用样条曲线拟合的方式计算的方法
function analyze_signals_(t,data, valid_flag,idx_unique, ax0, ay0, a_scalar0, alpha0)
    % 提取数据
    ax0 = ax0(valid_flag(idx_unique));
    ay0 = ay0(valid_flag(idx_unique));
    a_scalar0 = a_scalar0(valid_flag(idx_unique));
    alpha0 = alpha0(valid_flag(idx_unique));
    
    t = t(valid_flag);
    vx_vy_omega = data.signals.values(:, [2 4 5]);
    vx_vy_omega = vx_vy_omega(valid_flag, :);
    
    [t_unique_and_valid, idx_unique_and_valid] = unique(t, 'stable');
    vx_vy_omega_unique = vx_vy_omega(idx_unique_and_valid, :);

    
    % 提取vx, vy, omega
    vx = vx_vy_omega_unique(:, 1);
    vy = vx_vy_omega_unique(:, 2);
    omega = vx_vy_omega_unique(:, 3);
    
    % 使用cubic三次样条拟合vx, vy, omega
    pchip_vx = pchip(t_unique_and_valid, vx);
    pchip_vy = pchip(t_unique_and_valid, vy);
    pchip_omega = pchip(t_unique_and_valid, omega);
    
    % 计算出ax, ay, a_scalar, alpha
    pchip_ax = fnder(pchip_vx);
    pchip_ay = fnder(pchip_vy);
    pchip_alpha = fnder(pchip_omega);
    
    % 定义新的时间向量，用于更平滑的曲线绘制
    t_smooth = linspace(min(t_unique_and_valid), max(t_unique_and_valid), 1000000);
    
    % 计算平滑后的拟合值
    vx_fit = fnval(pchip_vx, t_smooth);
    vy_fit = fnval(pchip_vy, t_smooth);
    omega_fit = fnval(pchip_omega, t_smooth);
    
    ax = fnval(pchip_ax, t_unique_and_valid);
    ay = fnval(pchip_ay, t_unique_and_valid);
    alpha = fnval(pchip_alpha, t_unique_and_valid);
    
    % 计算原始数据的加速度和角加速度
    v = sqrt(vx.^2 + vy.^2);
    a_scalar = (vx .* ax + vy .* ay) ./ v;
    
    % 计算残差
    a_residual = a_scalar0 - a_scalar;
    alpha_residual = alpha0 - alpha;
    
    % 计算残差的方差
    a_residual_var = var(a_residual);
    alpha_residual_var = var(alpha_residual);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 绘制原始值与拟合曲线对比图
    figure;
    subplot(3, 1, 1);
    plot(t_unique_and_valid, vx, 'b', t_smooth, vx_fit, 'r');
    title('vx: Original vs Fitted');
    legend('Original', 'Fitted');
    
    subplot(3, 1, 2);
    plot(t_unique_and_valid, vy, 'b', t_smooth, vy_fit, 'r');
    title('vy: Original vs Fitted');
    legend('Original', 'Fitted');
    
    subplot(3, 1, 3);
    plot(t_unique_and_valid, omega, 'b', t_smooth, omega_fit, 'r');
    title('omega: Original vs Fitted');
    legend('Original', 'Fitted');
    
    % 绘制原来的值ax0, ay0, a_scalar0, alpha0与拟合的值ax, ay, a_scalar, alpha的对比图和残差图
    figure;
    subplot(4, 2, 1);
    plot(t_unique_and_valid, ax0, 'b', t_unique_and_valid, ax, 'r');
    title('ax: Predicted vs Fitted');
    legend('Predicted', 'Fitted');
    xlabel('Time (s)');
    ylabel('a_x (m/s²)');
    
    subplot(4, 2, 2);
    plot(t_unique_and_valid, ax0 - ax, 'g');
    title(['ax Residuals (Variance: ' num2str(var(ax0 - ax)) ')']);
    xlabel('Time (s)');
    ylabel('Residuals (m/s²)');
    
    subplot(4, 2, 3);
    plot(t_unique_and_valid, ay0, 'b', t_unique_and_valid, ay, 'r');
    title('ay: Predicted vs Fitted');
    legend('Predicted', 'Fitted');
    xlabel('Time (s)');
    ylabel('a_y (m/s²)');
    
    subplot(4, 2, 4);
    plot(t_unique_and_valid, ay0 - ay, 'g');
    title(['ay Residuals (Variance: ' num2str(var(ay0 - ay)) ')']);
    xlabel('Time (s)');
    ylabel('Residuals (m/s²)');
    
    subplot(4, 2, 5);
    plot(t_unique_and_valid, a_scalar0, 'b', t_unique_and_valid, a_scalar, 'r');
    title('a_scalar: Predicted vs Fitted');
    legend('Predicted', 'Fitted');
    xlabel('Time (s)');
    ylabel('a_scalar (m/s²)');
    
    subplot(4, 2, 6);
    plot(t_unique_and_valid, a_residual, 'g');
    title(['a_scalar Residuals (Variance: ' num2str(a_residual_var) ')']);
    xlabel('Time (s)');
    ylabel('Residuals (m/s²)');
    
    subplot(4, 2, 7);
    plot(t_unique_and_valid, alpha0, 'b', t_unique_and_valid, alpha, 'r');
    title('alpha: Predicted vs Fitted');
    legend('Predicted', 'Fitted');
    xlabel('Time (s)');
    ylabel('\alpha (rad/s^2)');
    
    subplot(4, 2, 8);
    plot(t_unique_and_valid, alpha_residual, 'g');
    title(['alpha Residuals (Variance: ' num2str(alpha_residual_var) ')']);
    xlabel('Time (s)');
    ylabel('Residuals (rad/s^2)');
end

% 使用中央差分公式的计算方法
function analyze_signals__(t, data, valid_flag, idx_unique, ax0, ay0, a_scalar0, alpha0)
    % 提取数据
    ax0 = ax0(valid_flag(idx_unique));
    ay0 = ay0(valid_flag(idx_unique));
    a_scalar0 = a_scalar0(valid_flag(idx_unique));
    alpha0 = alpha0(valid_flag(idx_unique));
    
    t = t(valid_flag);
    vx_vy_omega = data.signals.values(:, [2 4 5]);
    vx_vy_omega = vx_vy_omega(valid_flag, :);
    
    [t_unique_and_valid, idx_unique_and_valid] = unique(t, 'stable');
    vx_vy_omega_unique = vx_vy_omega(idx_unique_and_valid, :);

    % 提取vx, vy, omega
    vx = vx_vy_omega_unique(:, 1);
    vy = vx_vy_omega_unique(:, 2);
    omega = vx_vy_omega_unique(:, 3);
    
%     % 使用pchip三次样条拟合vx, vy, omega
%     pchip_vx = pchip(t_unique_and_valid, vx);
%     pchip_vy = pchip(t_unique_and_valid, vy);
%     pchip_omega = pchip(t_unique_and_valid, omega);
%     
%     % 定义新的时间向量，用于更平滑的曲线绘制
%     t_smooth = linspace(min(t_unique_and_valid), max(t_unique_and_valid), 1000);
%     
%     % 计算平滑后的拟合值
%     vx_fit = fnval(pchip_vx, t_smooth);
%     vy_fit = fnval(pchip_vy, t_smooth);
%     omega_fit = fnval(pchip_omega, t_smooth);
    
    % 使用中央差分公式计算ax, ay, alpha
%     dt = diff(t_unique_and_valid); % 计算时间间隔
%     dt = [dt(1); dt; dt(end)]; % 在向量两端补充，使长度一致
    
    ax = zeros(size(vx));
    ay = zeros(size(vy));
    alpha = zeros(size(omega));
    
    ax(2:end-1) = (vx(3:end) - vx(1:end-2)) ./ (t_unique_and_valid(3:end) - t_unique_and_valid(1:end-2));
    ay(2:end-1) = (vy(3:end) - vy(1:end-2)) ./ (t_unique_and_valid(3:end) - t_unique_and_valid(1:end-2));
    alpha(2:end-1) = (omega(3:end) - omega(1:end-2)) ./ (t_unique_and_valid(3:end) - t_unique_and_valid(1:end-2));
    
    % 边界处理，使用向前差分和向后差分
    ax(1) = (vx(2) - vx(1)) / (t_unique_and_valid(2) - t_unique_and_valid(1));
    ax(end) = (vx(end) - vx(end-1)) / (t_unique_and_valid(end) - t_unique_and_valid(end-1));
    
    ay(1) = (vy(2) - vy(1)) / (t_unique_and_valid(2) - t_unique_and_valid(1));
    ay(end) = (vy(end) - vy(end-1)) / (t_unique_and_valid(end) - t_unique_and_valid(end-1));
    
    alpha(1) = (omega(2) - omega(1)) / (t_unique_and_valid(2) - t_unique_and_valid(1));
    alpha(end) = (omega(end) - omega(end-1)) / (t_unique_and_valid(end) - t_unique_and_valid(end-1));
    
    % 计算原始数据的加速度和角加速度
    v = sqrt(vx.^2 + vy.^2);
    a_scalar = (vx .* ax + vy .* ay) ./ v;
    
    % 计算残差
    a_residual = a_scalar0 - a_scalar;
    alpha_residual = alpha0 - alpha;
    
    % 计算残差的方差
    a_residual_var = var(a_residual);
    alpha_residual_var = var(alpha_residual);
    
%     % 绘制原始值与拟合曲线对比图
%     figure;
%     subplot(3, 1, 1);
%     plot(t_unique_and_valid, vx, 'b', t_smooth, vx_fit, 'r');
%     title('vx: Original vs Fitted');
%     legend('Original', 'Fitted');
%     
%     subplot(3, 1, 2);
%     plot(t_unique_and_valid, vy, 'b', t_smooth, vy_fit, 'r');
%     title('vy: Original vs Fitted');
%     legend('Original', 'Fitted');
%     
%     subplot(3, 1, 3);
%     plot(t_unique_and_valid, omega, 'b', t_smooth, omega_fit, 'r');
%     title('omega: Original vs Fitted');
%     legend('Original', 'Fitted');
    
    % 绘制原来的值ax0, ay0, a_scalar0, alpha0与拟合的值ax, ay, a_scalar, alpha的对比图和残差图
    figure;
    subplot(4, 2, 1);
    plot(t_unique_and_valid, ax0, 'b', t_unique_and_valid, ax, 'r');
    title('ax: Predicted vs Fitted');
    legend('Predicted', 'Fitted');
    xlabel('Time (s)');
    ylabel('a_x (m/s²)');

    subplot(4, 2, 2);
    plot(t_unique_and_valid, ax0 - ax, 'g');
    title(['ax Residuals (Variance: ' num2str(var(ax0 - ax)) ')']);
    xlabel('Time (s)');
    ylabel('Residuals (m/s²)');
    
    subplot(4, 2, 3);
    plot(t_unique_and_valid, ay0, 'b', t_unique_and_valid, ay, 'r');
    title('ay: Predicted vs Fitted');
    legend('Predicted', 'Fitted');
    xlabel('Time (s)');
    ylabel('a_y (m/s²)');
    
    subplot(4, 2, 4);
    plot(t_unique_and_valid, ay0 - ay, 'g');
    title(['ay Residuals (Variance: ' num2str(var(ay0 - ay)) ')']);
    xlabel('Time (s)');
    ylabel('Residuals (m/s²)');
    
    subplot(4, 2, 5);
    plot(t_unique_and_valid, a_scalar0, 'b', t_unique_and_valid, a_scalar, 'r');
    title('a_scalar: Predicted vs Fitted');
    legend('Predicted', 'Fitted');
    xlabel('Time (s)');
    ylabel('a_scalar (m/s²)');
    
    subplot(4, 2, 6);
    plot(t_unique_and_valid, a_residual, 'g');
    title(['a_scalar Residuals (Variance: ' num2str(a_residual_var) ')']);
    xlabel('Time (s)');
    ylabel('Residuals (m/s²)');
    
    subplot(4, 2, 7);
    plot(t_unique_and_valid, alpha0, 'b', t_unique_and_valid, alpha, 'r');
    title('alpha: Predicted vs Fitted');
    legend('Predicted', 'Fitted');
    xlabel('Time (s)');
    ylabel('\alpha (rad/s^2)');
    
    subplot(4, 2, 8);
    plot(t_unique_and_valid, alpha_residual, 'g');
    title(['alpha Residuals (Variance: ' num2str(alpha_residual_var) ')']);
    xlabel('Time (s)');
    ylabel('Residuals (rad/s^2)');
end

function analyze_signals(t, data, valid_flag, idx_unique, ax0, ay0, a_scalar0, alpha0)
    % 提取数据
    ax0 = ax0(valid_flag(idx_unique));
    ay0 = ay0(valid_flag(idx_unique));
    a_scalar0 = a_scalar0(valid_flag(idx_unique));
    alpha0 = alpha0(valid_flag(idx_unique));
    
    t = t(valid_flag);
    vx_vy_omega = data.signals.values(:, [2 4 5]);
    vx_vy_omega = vx_vy_omega(valid_flag, :);
    
    [t_unique_and_valid, idx_unique_and_valid] = unique(t, 'stable');
    vx_vy_omega_unique = vx_vy_omega(idx_unique_and_valid, :);

    % 提取vx, vy, omega
    vx = vx_vy_omega_unique(:, 1);
    vy = vx_vy_omega_unique(:, 2);
    omega = vx_vy_omega_unique(:, 3);

    % 使用四阶差分公式计算ax, ay, alpha
    ax = zeros(size(vx));
    ay = zeros(size(vy));
    alpha = zeros(size(omega));
    
    % 四阶差分公式的计算范围是2:end-1
    for i = 3:length(vx)-2
        four_h = t_unique_and_valid(i+2) - t_unique_and_valid(i-2);
        ax(i) = (-vx(i+2) + 8*vx(i+1) - 8*vx(i-1) + vx(i-2)) / (3*four_h);
        ay(i) = (-vy(i+2) + 8*vy(i+1) - 8*vy(i-1) + vy(i-2)) / (3*four_h);
        alpha(i) = (-omega(i+2) + 8*omega(i+1) - 8*omega(i-1) + omega(i-2)) / (3*four_h);
    end
    
    % 边界处理，使用向前差分和向后差分
    ax(1:2) = (vx(2:3) - vx(1:2)) ./ (t_unique_and_valid(2:3) - t_unique_and_valid(1:2));
    ax(end-1:end) = (vx(end-1:end) - vx(end-2:end-1)) ./ (t_unique_and_valid(end-1:end) - t_unique_and_valid(end-2:end-1));
    
    ay(1:2) = (vy(2:3) - vy(1:2)) ./ (t_unique_and_valid(2:3) - t_unique_and_valid(1:2));
    ay(end-1:end) = (vy(end-1:end) - vy(end-2:end-1)) ./ (t_unique_and_valid(end-1:end) - t_unique_and_valid(end-2:end-1));
    
    alpha(1:2) = (omega(2:3) - omega(1:2)) ./ (t_unique_and_valid(2:3) - t_unique_and_valid(1:2));
    alpha(end-1:end) = (omega(end-1:end) - omega(end-2:end-1)) ./ (t_unique_and_valid(end-1:end) - t_unique_and_valid(end-2:end-1));

    % 计算原始数据的加速度和角加速度
    v = sqrt(vx.^2 + vy.^2);
    a_scalar = (vx .* ax + vy .* ay) ./ v;
    
    % 计算残差
    a_residual = a_scalar0 - a_scalar;
    alpha_residual = alpha0 - alpha;
    
    % 计算残差的方差
    a_residual_var = var(a_residual);
    alpha_residual_var = var(alpha_residual);
    
    % 绘制原来的值ax0, ay0, a_scalar0, alpha0与拟合的值ax, ay, a_scalar, alpha的对比图和残差图
    figure;
    subplot(4, 2, 1);
    plot(t_unique_and_valid, ax0, 'b', t_unique_and_valid, ax, 'r');
    title('ax: Predicted vs Fitted');
    legend('Predicted', 'Fitted');
    xlabel('Time (s)');
    ylabel('a_x (m/s²)');
    
    subplot(4, 2, 2);
    plot(t_unique_and_valid, ax0 - ax, 'g');
    title(['ax Residuals (Variance: ' num2str(var(ax0 - ax)) ')']);
    xlabel('Time (s)');
    ylabel('Residuals (m/s²)');
    
    subplot(4, 2, 3);
    plot(t_unique_and_valid, ay0, 'b', t_unique_and_valid, ay, 'r');
    title('ay: Predicted vs Fitted');
    legend('Predicted', 'Fitted');
    xlabel('Time (s)');
    ylabel('a_y (m/s²)');
    
    subplot(4, 2, 4);
    plot(t_unique_and_valid, ay0 - ay, 'g');
    title(['ay Residuals (Variance: ' num2str(var(ay0 - ay)) ')']);
    xlabel('Time (s)');
    ylabel('Residuals (m/s²)');
    
    subplot(4, 2, 5);
    plot(t_unique_and_valid, a_scalar0, 'b', t_unique_and_valid, a_scalar, 'r');
    title('a_scalar: Predicted vs Fitted');
    legend('Predicted', 'Fitted');
    xlabel('Time (s)');
    ylabel('a_scalar (m/s²)');
    
    subplot(4, 2, 6);
    plot(t_unique_and_valid, a_residual, 'g');
    title(['a_scalar Residuals (Variance: ' num2str(a_residual_var) ')']);
    xlabel('Time (s)');
    ylabel('Residuals (m/s²)');
    
    subplot(4, 2, 7);
    plot(t_unique_and_valid, alpha0, 'b', t_unique_and_valid, alpha, 'r');
    title('alpha: Predicted vs Fitted');
    legend('Predicted', 'Fitted');
    xlabel('Time (s)');
    ylabel('\alpha (rad/s^2)');
    
    subplot(4, 2, 8);
    plot(t_unique_and_valid, alpha_residual, 'g');
    title(['alpha Residuals (Variance: ' num2str(alpha_residual_var) ')']);
    xlabel('Time (s)');
    ylabel('Residuals (rad/s^2)');
end

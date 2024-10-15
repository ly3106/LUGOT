fitted_x = fnval(sp_x, time_uwb(valid_raw & criterion_valid_));
fitted_y = fnval(sp_y, time_uwb(valid_raw & criterion_valid_));

% 调用函数
value1 = UWB_EKF_Rawd_CT_3D_All.signals.values(:, [1 3]);
value2 = UWB_UKF_Rawd_CT_3D_All.signals.values(:, [1 3]);
value3 = UWB_EKF_Rawd_CT_3D.signals.values(:, [1 3]);
value4 = UWB_UKF_Rawd_CT_3D.signals.values(:, [1 3]);
value5 = UWB_EKF_Rawd_CT.signals.values(:, [1 3]);
value6 = UWB_UKF_Rawd_CT.signals.values(:, [1 3]);
value7 = uwb_global_lidar;
value1 = value1(valid_raw & criterion_valid_,:);
value2 = value2(valid_raw & criterion_valid_,:);
value3 = value3(valid_raw & criterion_valid_,:);
value4 = value4(valid_raw & criterion_valid_,:);
value5 = value5(valid_raw & criterion_valid_,:);
value6 = value6(valid_raw & criterion_valid_,:);
value7 = value7(valid_raw & criterion_valid_,:);

plot_error_comparison(time_uwb(valid_raw & criterion_valid_), fitted_x, fitted_y, ...
    struct('values', uwb_raw_global_lidar(valid_raw & criterion_valid_,:), 'basicName', 'UWB Raw'), ...
    struct('values', value7, 'basicName', 'KF \itd_{\rm1}, d_{\rm2}'), ...
    struct('values', value1, 'basicName', 'EKF 3D All'), ...
    struct('values', value2, 'basicName', 'UKF 3D All'), ...
    struct('values', value3, 'basicName', 'EKF 3D (\bfours\rm)'), ...
    struct('values', value4, 'basicName', 'UKF 3D (\bfours\rm)'), ...
    struct('values', value5, 'basicName', 'EKF 2D'), ...
    struct('values', value6, 'basicName', 'UKF 2D'));

function plot_error_comparison(time_uwb, fitted_x, fitted_y, varargin)
    % 提取数据并存储在矩阵中
    num_datasets = length(varargin);
    num_points = length(time_uwb);
    
    x_errors = zeros(num_points, num_datasets);
    y_errors = zeros(num_points, num_datasets);
    distance_errors = zeros(num_points, num_datasets);
    basicNames = cell(1, num_datasets);
    
    for i = 1:num_datasets
        data = varargin{i};
        x = data.values(:,1);
        y = data.values(:,2);
        x_errors(:,i) = x - fitted_x;
        y_errors(:,i) = y - fitted_y;
        distance_errors(:,i) = sqrt((x - fitted_x).^2 + (y - fitted_y).^2);
        basicNames{i} = data.basicName;
    end
    
    mota = mean(distance_errors);
    fprintf('MOTA = %.4f\n', mota);

    % 绘制误差对比
    figure;
    tiledlayout(2, 2);

    % 绘制 x 误差对比
    nexttile;
    h1 = plot(time_uwb, x_errors);
    ylabel('Error (m)');
    title('x Error Comparison');
    
    % 绘制 y 误差对比
    nexttile;
    plot(time_uwb, y_errors);
    xlabel('Time (s)');
    ylabel('Error (m)');
    title('y Error Comparison');
    
    % 绘制距离误差对比
    nexttile;
    plot(time_uwb, distance_errors);
    xlabel('Time (s)');
    ylabel('Error (m)');
    title('Distance Error Comparison');
    
    % 创建一个空的子图用于图例
    lgd = nexttile;
    axis off; % 隐藏轴线
    legend(lgd, h1, basicNames, 'NumColumns', 1, 'Location', 'best');
    
    % 计算并输出 RMSE
    rmse_values = zeros(3, num_datasets);
    for i = 1:num_datasets
        rmse_x = sqrt(mean(x_errors(:,i).^2));
        rmse_y = sqrt(mean(y_errors(:,i).^2));
        rmse = sqrt(mean(distance_errors(:,i).^2));
        
        rmse_values(:,i) = [rmse; rmse_x; rmse_y];
        
%         fprintf('The RMSE for %s x coordinates is: %f\n', basicNames{i}, rmse_x);
%         fprintf('The RMSE for %s y coordinates is: %f\n', basicNames{i}, rmse_y);
%         fprintf('The RMSE for %s 2-D positioning is: %f\n', basicNames{i}, rmse);
    end
    
    % 创建柱状图
    figure;
    hBar = bar(rmse_values);
    hBar(8).FaceColor = [.2 .6 .5];
    legend(basicNames, 'NumColumns', 4, 'Location', 'northoutside'); % 添加图例
    ylabel('RMSE Value (m)'); % y轴标签
%     title('RMSE Comparison'); % 图的标题
    grid on;
%     grid minor;
    
    % 设置X轴的标签，例如，对应不同的测量或情况
    set(gca, 'XTickLabel', {'RMSE_{\itd}', 'RMSE_{\itx}', 'RMSE_{\ity}'});
    
    % 在每个柱子上方添加y标签
    % 遍历每一组柱子
    for i = 1:length(hBar)
        % 获取当前组的y数据
        ydata = hBar(i).YData;
        % 获取当前组的x数据
        xdata = hBar(i).XData + hBar(i).XOffset;
        % 在每个柱子上方添加文本
        for j = 1:length(ydata)
            text(xdata(j), ydata(j), num2str(ydata(j), '%.4f'), ...
                'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left','Rotation', 75);
        end
    end
end


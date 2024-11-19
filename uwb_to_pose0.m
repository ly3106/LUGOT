clear

run('data\plot_filtered_raw.m') % 计算UWB坐标系下的(x,y)，及其是否能构成三角形

% SLAM_FLAG = 'LOAM+LeGO-LOAM';
% SLAM_FLAG = 'CT-ICP2';
SLAM_FLAG = 'KISS-ICP';
% SLAM_FLAG = 'MOLA';
NUM_DIGITS_TO_REMOVE = 6;

% 定义路径映射表
slamFileMap = containers.Map( ...
    {'LOAM+LeGO-LOAM', 'CT-ICP2', 'KISS-ICP', 'MOLA'}, ...
    {'.\data\poses\pose_uwb_in_velodyne0_at_uwb_pose_stamped.xlsx', ...
     '.\data\poses\pose_uwb_in_velodyne0_ct_icp_at_uwb_anchors.xlsx', ...
     '.\data\poses\pose_uwb_in_velodyne0_kiss_icp_at_uwb_anchors.xlsx', ...
     '.\data\poses\pose_uwb_in_velodyne0_mola_at_uwb_anchors.xlsx'} ...
);

% 检查并读取对应表格文件
if isKey(slamFileMap, SLAM_FLAG)
    T = readtable(slamFileMap(SLAM_FLAG));
else
    error('ERROR: This SLAM is not defined!');
end

% 对齐时间戳,两个SLAM的时间戳交集
common_time_range = get_common_range_from_slamFileMap(slamFileMap);

time_trimmed = merge_timestamp_columns(T, NUM_DIGITS_TO_REMOVE);
is_within_time_range = time_trimmed >= common_time_range(1) & time_trimmed <= common_time_range(2);
T = T(is_within_time_range, :);
q = [T.orientation_w, T.orientation_x, T.orientation_y, T.orientation_z];
eul_rad = quat2eul(q, 'ZYX');

T1 = readtable('data\PstnUWB.xlsx');
time_trimmed = merge_timestamp_columns(T1, NUM_DIGITS_TO_REMOVE);
is_within_time_range = time_trimmed >= common_time_range(1) & time_trimmed <= common_time_range(2);
T1 = T1(is_within_time_range, :);

valid_raw = valid_raw(is_within_time_range, :);
x_raw = x_raw(is_within_time_range, :);
y_raw = y_raw(is_within_time_range, :);
RawD1m = RawD1m(is_within_time_range, :);
RawD2m = RawD2m(is_within_time_range, :);

position = table2array(T1(:, {'Time_s_', 'x_m_', 'y_m_'}));
time_uwb = position(:,1);

% 计算角度
theta = eul_rad(:,1);

% 计算cos和sin值
cos_theta = cos(theta);
sin_theta = sin(theta);

% 初始化旋转矩阵数组
R = zeros(2, 2, length(theta));

% 填充旋转矩阵数组
R(1,1,:) = cos_theta;
R(1,2,:) = -sin_theta;
R(2,1,:) = sin_theta;
R(2,2,:) = cos_theta;

% 自车位置
lidar_pos = [T.position_x,T.position_y]; % LiDAR/UWB的原点位置

% 前车相对于自车的位置
rel_pos = position(:,2:3);
% 前车在世界坐标系中的位置,通过LiDAR/UWB位置计算
% 对每个时间点应用旋转矩阵和位移
uwb_global_lidar = zeros(size(rel_pos));
for i = 1:size(rel_pos, 1)
    uwb_global_lidar(i,:) = lidar_pos(i,:) + (squeeze(R(:,:,i)) * rel_pos(i,:)')';
end

% 前车Raw值相对于自车的位置
rel_pos_raw = [x_raw(:,1), y_raw];

% 前车在世界坐标系中的位置,通过LiDAR/UWB位置计算
% 对每个时间点应用旋转矩阵和位移
uwb_raw_global_lidar = zeros(size(rel_pos_raw));
for i = 1:size(rel_pos_raw, 1)
    uwb_raw_global_lidar(i,:) = lidar_pos(i,:) + (squeeze(R(:,:,i)) * rel_pos_raw(i,:)')';
end

if length(dbstack) == 1
    figure
    plot(uwb_global_lidar(:,1), uwb_global_lidar(:,2),'.-','DisplayName','uwb\_global\_lidar')
    hold on;
    x = uwb_raw_global_lidar(:,1);
    y = uwb_raw_global_lidar(:,2);
    plot(x, y,'.-','DisplayName','uwb\_raw\_global\_lidar')
    scatter(x(~valid_raw),y(~valid_raw), 'DisplayName','Non-triangular')
    axis equal; % 设置x轴和y轴的比例尺相同
    axis tight; % 调整坐标轴的范围以紧凑地显示图形
    xlabel('x (m)')
    ylabel('y (m)')
    legend show;
    hold off
end

clear T T1 q i eul_rad time_trimmed sin_theta cos_theta R rel_pos rel_pos_raw

function common_range = get_common_timestamps(t1, t2)
    % GET_COMMON_TIMESTAMPS 获得两个时间戳列的范围交集
    % 输入:
    %   t1 - 第一个以秒为单位的浮点数时间戳列 (列向量)
    %   t2 - 第二个以秒为单位的浮点数时间戳列 (列向量)
    % 输出:
    %   common_range - 两个输入时间戳列的范围交集

    % 找到每个时间戳列的最小和最大值
    min_t1 = min(t1);
    max_t1 = max(t1);
    min_t2 = min(t2);
    max_t2 = max(t2);

    % 计算两个范围的交集
    common_start = max(min_t1, min_t2);
    common_end = min(max_t1, max_t2);

    % 如果范围有效（即 common_start <= common_end），则返回范围
    if common_start <= common_end
        common_range = [common_start, common_end];
    else
        common_range = [];
    end

end

function common_range = get_common_range_from_slamFileMap(slamFileMap)
    % GET_COMMON_RANGE_FROM_SLAMFILEMAP 遍历 slamFileMap，获取时间戳范围交集
    % 输入:
    %   slamFileMap - 包含文件路径的 containers.Map
    % 输出:
    %   common_range - 所有文件时间戳列的范围交集

    keys = slamFileMap.keys;
    time_stamps = cell(1, numel(keys));

    for i = 1:numel(keys)
        % 读取文件并获取时间戳列
        filePath = slamFileMap(keys{i});
        T = readtable(filePath);
        time_stamps{i} = T.time_trimmed;
    end

    % 计算所有时间戳列的范围交集
    common_range = time_stamps{1};
    for i = 2:numel(time_stamps)
        common_range = get_common_timestamps(common_range, time_stamps{i});
        if isempty(common_range)
            error('ERROR: Commom time rage is empty!');
        end
    end

end

function timestamp_s_float = merge_timestamp_columns(T, num_digits_to_remove)
    % MERGE_TIMESTAMP_COLUMNS 将 table 型的 timestamp_s 列去掉前面 num_digits_to_remove 个数字后，
    % 与 timestamp_ns 列合并成以秒为单位的浮点型时间戳
    % 输入:
    %   T - 包含 timestamp_s 和 timestamp_ns 列的 table
    %   num_digits_to_remove - 去掉 timestamp_s 列中前面的数字个数
    % 输出:
    %   timestamp_s_float - 合并后的浮点型时间戳，以秒为单位

    % 去掉 timestamp_s 列前面 num_digits_to_remove 个数字
    timestamp_s_trimmed = mod(T.timestamp_s, 10^(floor(log10(T.timestamp_s(1))) + 1 - num_digits_to_remove));

    % 将秒和纳秒部分合并为浮点型时间戳
    timestamp_s_float = timestamp_s_trimmed + T.timestamp_ns * 1e-9;

end
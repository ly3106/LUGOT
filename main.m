clc
clear

run('uwb_to_pose0.m')
load('data\hyper_parameter.mat')

Ts = 1.; % 主要用于Simulink仿真步长计数，不是真的周期
num_init_steps = uint8(3); % 初始化步长的个数，用于使*KF内部参数完成收敛
standerd_deviation = [0.03727 0.03621];
outlier_threshold = standerd_deviation * 3.75 * 2 + 0.0;
cov_process_noise = diag([32 128 16])/16.;
cov_measurement_noise = diag(standerd_deviation.^2);

% 将 SLAM_FLAG 转换为大写，避免大小写敏感问题
slamFlag = upper(SLAM_FLAG);
% 逻辑判断并读取对应的文件
if strcmp(slamFlag, 'LOAM+LEGO-LOAM')
    init_state = [23.43; 0.2; 0.46; 0.2; 0.001]; % [28; 0.2; 0.01; 0.2; 0.001]
elseif strcmp(slamFlag, 'CT-ICP2')
    init_state = [26.8319; 0.844; 0.7445; 0.2; -0.50576]; % [27; 0.2; 0.01; 0.2; 0.001]
else
    error('ERROR: This SLAM is not defined!');
end

open_system('LUGOT.slx');
sim('LUGOT.slx');

run('smoothing_spline_fit.m')
run('calc_process_noise.m')
run('plot_resault.m')
run('plot_statistics.m')
% run('relative_spd_vs_residual')
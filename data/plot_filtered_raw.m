% 计算UWB原始值在前车UWB坐标系下的位置，考虑高度差因素

% 基站0的坐标为(0, -a)，基站1的坐标为(0,a)，d1为标签到基站0的距离，d2为标签到基
% 站1的距离，这个函数考虑了三角定位过程中d1、d2、2a构成的目标点与两个基站在同一
% 直线上的情况，这种情况也给出一个参考值，并且valid依然为Nan。当构成不了三角形时，
% 判断是位于两基站左侧还是右侧还是中间，然后计算对应的由d1得到的位置、d2得到的位
% 置（x肯定都为0），然后将这两个位置求平均值，作为一个猜测位置

% clear

load('hyper_parameter.mat');
% 加载CSV文件，假设文件名为'data.csv'，并假设数据有标题行
data = readtable('UWB_Timestamp_Distance.xlsx');

% 提取数据
RawD1m = data{:, 3};
RawD2m = data{:, 4};

% 初始化输出数组
x_raw = [];
y_raw = [];
valid_raw = logical([]);

% 循环处理每一对d1和d2
for i = 1:length(RawD1m)
    [x, y, valid] = calculateXY(RawD1m(i), RawD2m(i), a, delta_h);
    x_raw = [x_raw; x];     % 保存x值
    y_raw = [y_raw; y];     % 保存y值
    valid_raw = [valid_raw; valid];  % 保存有效性标志
end

% 检查是否直接运行
if length(dbstack) == 1
    disp(['脚本被直接运行，所在路径为: ', mfilename('fullpath')]);
    % 仅在直接运行时执行的代码
    % 绘制图形
    x = x_raw(:,1);
    y = y_raw;
    figure; % 创建一个新图形窗口
    plot(x, y, '.-'); % 绘制线图，数据点用圆圈表示
    axis equal; % 设置x轴和y轴的比例尺相同
    xlabel('x (m)'); % 设置x轴标签为第二列的标题
    ylabel('y (m)'); % 设置y轴标签为第三列的标题
    title('Plot of y versus x'); % 设置图形的标题
    grid on; % 开启网格
else
    disp(['脚本被调用，所在路径为: ', mfilename('fullpath')]); 
end

clearvars -except RawD1m RawD2m x_raw y_raw valid_raw;
% save('output_data/xy_raw.mat', 'RawD1m', 'RawD2m', 'x_raw', 'y_raw', 'valid_raw');

% 三角定位函数
function [x, y, valid] = calculateXY(d1, d2, a, delta_h)
    % 检查是否能构成三角形
    if (d1 + d2 > 2*a) && (d1 + 2*a > d2) && (d2 + 2*a > d1)
        valid = true;
        % 计算y
        y = (d1^2 - d2^2) / (4 * a);
        
        % 计算x的两个可能值
        try
            x1 = sqrt(d1^2 - (y + a)^2 + delta_h^2);
            x2 = -x1;
            x = [x1, x2];  % 返回两个可能的x值
        catch
            % 如果x的计算中出现错误（比如求平方根的负数），标记为无效
            valid = false;
            x = [NaN, NaN];
            y = NaN;
            error('解三角形出错');
        end
    else
        % 不能构成三角形，返回无效标志
        valid = false;
%         x = [NaN, NaN];
%         y = NaN;
        % 判断标签的大概位置并计算y值
        if (d1 + d2) <= 2*a
            % 标签在两基站的中间
            y1 = - a + d1;
            y2 = a - d2;
            if (d1 + d2) == 2*a
                valid = true;
            end
        elseif (d2 +2*a) <= d1
            % 标签在两基站的左侧
            y1 = -a + d1;
            y2 = a + d2;
            if (d2 +2*a) == d1
                valid = true;
            end
        elseif (d1 + 2*a) <= d2
            % 标签在两基站的右侧
            y1 = -a - d1;
            y2 = a - d2;
            if (d1 + 2*a) == d2
                valid = true;
            end
        else
            error('判断分类没有全部覆盖');
        end
        
        % 计算平均y值
        y = (y1 + y2) / 2;
        x = [0,0];  % x位置肯定为0
    end
end

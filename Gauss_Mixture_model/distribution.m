%% 高斯混合模型处理残差权重
clc
clear all
close all

%% 参数定义
global RA;
global E2;
global D2R;
RA = 6378137;          % WGS84长半轴[3](@ref)
E2 = 0.00669437999013; % 第一偏心率的平方
D2R = pi/180;          % 角度转弧度

%% 主运行流程
%% 读取数据
% GPS数据转换并绘图
% 读取整个文件
T = readtable('gnss_true_data_urban38.txt',  'Delimiter',  ',',  'ReadVariableNames',  false); % gps_output.txt gnss_data.csv
% 删除第4列（文本列）
T(:, 4) = [];
navData = table2array(T); % 转换为数值矩阵
GPS = GPS_transform(navData); % 1:3为xyz位置，4:6为xyz标准差

% 读入真值轨迹数据
gt_path = 'truth_traj.csv';
gt_raw = readtable(gt_path,  'Delimiter',  ' ',  'ReadVariableNames',  false);
gt_data = table2array(gt_raw); % 转换为数值矩阵
gt_timestamps = gt_data(:, 1) - gt_data(1, 1);
gt_xyz = gt_data(:, 2:4);
gt = [gt_timestamps, gt_xyz]; 

%% 轨迹对齐
% 时间戳对齐
[GPS_sync, gt_sync] = synchronize_custom(GPS, gt, 0.01);
GPS_sync_xyz = GPS_sync(:, 1:3); % GPS_sync包含标准差

% ICP对齐
[R, t, s] = umeyama_alignment(gt_sync', GPS_sync_xyz');
gt_align = (R * gt_sync' + t)';

%% GMM拟合
% 根据给出的std计算分配权重误差
err = GPS_sync_xyz - gt_align;
% weight_err = err .* (1 ./ sqrt(GPS_sync(:, 4:6)));
weight_err = err;

% 第二个值为拟合高斯分布阶数
[post_prob, gmm1, gmm2, gmm3] = GMM_fit(weight_err, 2); 
weight = post_prob; % 权重和正常值的后验概率相同
reweighted_err = weight .* weight_err; % 重新分配误差权重
a = weight * 100;

%% 可视化
% 对齐轨迹绘图
traj_plot(GPS_sync_xyz);
traj_plot(gt_align);

% 误差绘图
err_plot(weight_err, reweighted_err);

% GMM相关参数绘图
GMM_plot(weight_err, gmm1, gmm2, gmm3);

%% GPS转换到ENU坐标系函数
function [traj] = GPS_transform(navData)
global RA;
global E2;
global D2R;

ref_time = navData(1, 1); % 起始时间
ref_pos = navData(1,  6:8); % 参考点[纬度,  经度,  高度](单位：度)

% 预处理
time_stamps = navData(:, 1);
lat = navData(:,  6) * D2R;    % 纬度转弧度
lon = navData(:,  7) * D2R;    % 经度转弧度
hgt = navData(:,  8);          % 高程(m)
std_x = navData(:,  9);        % 标准差
std_y = navData(:,  13); 
std_z = navData(:,  17); 

% 计算局部坐标
nPoints = size(navData,  1);
traj = zeros(nPoints,  7);

for i = 1:nPoints
    % [x, y, z] = blh2xyz(lat(i), lon(i), hgt(i));
    % 
    % % 添加偏移
    % x_new = x + 0.06 * i;
    % y_new = y + 0.06 * i;
    % z_new = z + 0 * i;

    % % 转换回BLH
    % [lat(i), lon(i), hgt(i)] = xyz2blh(x_new, y_new, z_new);

    % 计算曲率半径[3](@ref)
    sinLat = sin(lat(i));
    Rm = RA*(1-E2) / (1 - E2*sinLat^2)^1.5;
    Rn = RA / sqrt(1 - E2*sinLat^2);

    % 坐标差计算
    dLat = lat(i) - ref_pos(1)*D2R; % 
    dLon = lon(i) - ref_pos(2)*D2R; % 

    % ENU坐标转换[5](@ref)
    traj(i, 1) = (time_stamps(i) - ref_time) / 1e9;
    traj(i, 2) = (Rn + hgt(i)) * dLon * cos(lat(i)); % 东向
    traj(i, 3) = (Rm + hgt(i)) * dLat;               % 北向
    traj(i, 4) = hgt(i) - ref_pos(3);                % 天向 
end

traj(:,5:7) = [std_x, std_y, std_z];

end

%% 轨迹对齐函数
function [R, t, s] = umeyama_alignment(X, Y)
    % 输入参数：
    % X: 源点集，形状为 d×n（d为维度，n为点数）
    % Y: 目标点集，形状为 d×n
    % 输出参数：
    % R: 旋转矩阵（d×d）
    % t: 平移向量（d×1）
    % s: 缩放因子

    % 检查维度与点数一致性
    if size(X, 2) ~= size(Y, 2)
        error('点集数量不一致');
    end
    d = size(X, 1);
    n = size(X, 2);

    % 步骤1：计算质心
    centroid_X = mean(X, 2);
    centroid_Y = mean(Y, 2);

    % 步骤2：去质心化
    X_centered = X - centroid_X;
    Y_centered = Y - centroid_Y;

    % 步骤3：计算协方差矩阵
    H = (1/n) * (X_centered * Y_centered');

    % 步骤4：SVD分解
    [U, S, V] = svd(H);

    % 步骤5：计算旋转矩阵（考虑反射情况）
    det_sign = det(V * U');
    if det_sign < 0
        V(:, d) = -V(:, d);  % 修正反射
    end
    R = V * U';

    % 步骤6：计算缩放因子
    var_X = sum(var(X_centered, 0, 2));  % 源点集方差
    scale_factor = trace(S) / var_X;
    s = scale_factor;

    % 步骤7：计算平移向量
    t = centroid_Y - s * R * centroid_X;
end

%% 时间对齐函数
function [lowFreqData_sync, highFreqData_sync] = synchronize_custom(lowFreqData, highFreqData, threshold)
    % 提取时间戳列
    t_low = lowFreqData(:,1);  % 转换为datetime类型
    t_high = highFreqData(:,1);
    
    % 初始化同步矩阵
    highFreqData_sync = zeros(size(lowFreqData,1), size(highFreqData,2));
    
    % 遍历低频时间戳
    for i = 1:length(t_low)
        % 计算所有高频时间与当前低频时间的差值
        timeDiff = t_high - t_low(i);  % 转换为秒
        
        % 查找阈值范围内的索引[6](@ref)
        validIdx = find(abs(timeDiff) <= threshold);
        
        % 记录最近的有效时间（或取平均值）
        if ~isempty(validIdx)
            [~, minIdx] = min(abs(timeDiff(validIdx)));
            highFreqData_sync(i,:) = highFreqData(validIdx(minIdx), :);
        else
            highFreqData_sync(i,:) = NaN;  % 无匹配时填充NaN
        end
    end
    
    nan_mask = any(isnan(highFreqData_sync(:,2:end)), 2);  % 标记NaN为1

    % highFreqData_sync = [lowFreqData(:,1), highFreqData_sync]; % 合并低频时间戳到第一列，方便核对

    % 剔除查找不到的时间数据
    highFreqData_sync = highFreqData_sync(~nan_mask, :);
    lowFreqData_sync = lowFreqData(~nan_mask, :);

    % 提取三维数据
    highFreqData_sync = highFreqData_sync(:, 2:4);
    lowFreqData_sync = lowFreqData_sync(:, 2:7); % 加上三维std
end

%% GMM模型拟合函数
function [post_prob, gmm1, gmm2, gmm3] = GMM_fit(err, degree)
post_prob = zeros(size(err, 1), size(err, 2));
% 模型拟合
options = statset('Display', 'final', 'MaxIter', 1000);
gmm1 = fitgmdist(err(:, 1), degree, 'CovarianceType', 'diagonal', 'Options', options);
gmm2 = fitgmdist(err(:, 2), degree, 'CovarianceType', 'diagonal', 'Options', options);
gmm3 = fitgmdist(err(:, 3), degree, 'CovarianceType', 'diagonal', 'Options', options);

% 计算后验概率 
% [~, col_idx] = max(post_prob(:, 1:2), [], 2); % 返回最大后验的分类
post_prob1 = posterior(gmm1, err(:, 1));  % post_prob为m×K矩阵
post_prob2 = posterior(gmm2, err(:, 2));
post_prob3 = posterior(gmm3, err(:, 3));

post_prob(:, 1) = post_prob1(:, 1); % 第一列为正常分类
post_prob(:, 2) = post_prob2(:, 1);
post_prob(:, 3) = post_prob3(:, 1);

end

%% 绘图三维轨迹函数
function [] = traj_plot(traj)
figure(1);
plot3(traj(:, 1),  traj(:, 2),  traj(:, 3),  'LineWidth',  1.5);
xlabel('East (m)'); 
ylabel('North (m)'); 
zlabel('Up (m)');
title('三维轨迹图');
legend("GNSS", "ground_truth");
grid on;
axis equal;
hold on;

% 三个方向的数据
figure(2);
% 东向变化
subplot(3,  1,  1);
plot(traj(:, 1));
title('东向位移变化');
xlabel('时间/索引'); 
ylabel('位移(m)');
legend("GNSS", "ground_truth");
grid on;
hold on;

% 北向变化
subplot(3, 1, 2);
plot(traj(:, 2));
title('北向位移变化');
xlabel('时间/索引'); 
ylabel('位移(m)');
legend("GNSS", "ground_truth");
grid on;
hold on;

% 天向变化
subplot(3, 1, 3);
plot(traj(:, 3));
title('天向位移变化');
xlabel('时间/索引'); 
ylabel('位移(m)');
legend("GNSS", "ground_truth");
grid on;
hold on;

end

%% 绘制真值之间的误差轨迹以及分类结果函数
function [] = err_plot(traj, classification)
% 三个方向的数据
figure(3);
% 东向变化
subplot(3,  1,  1);
plot(traj(:, 1));
title('东向位移差值');
xlabel('时间/索引'); 
ylabel('位移(m)');
legend("GNSS", "ground_truth");
grid on;
hold on;
plot(classification(:, 1)');

% 北向变化
subplot(3, 1, 2);
plot(traj(:, 2));
title('北向位移差值');
xlabel('时间/索引'); 
ylabel('位移(m)');
legend("GNSS", "ground_truth");
grid on;
hold on;
plot(classification(:, 2)');

% 天向变化
subplot(3, 1, 3);
plot(traj(:, 3));
title('天向位移差值');
xlabel('时间/索引'); 
ylabel('位移(m)');
legend("GNSS", "ground_truth");
grid on;
hold on;
plot(classification(:, 3)');

end

%% GMM相关可视化
function [] = GMM_plot(err, gmm1, gmm2, gmm3)
figure(4);
x = linspace(min(err(:, 1)), max(err(:, 1)), 1000)';
histogram(err(:, 1), 'Normalization', 'pdf'); % 画出原数据概率直方图
hold on;
plot(x, pdf(gmm1, x), 'r-', 'LineWidth', 2); % 画出拟合的pdf
title('东向一维轨迹差值GMM拟合');

figure(5)
x = linspace(min(err(:, 2)), max(err(:, 2)), 1000)';
histogram(err(:, 2), 'Normalization', 'pdf'); % 画出原数据概率直方图
hold on;
plot(x, pdf(gmm2, x), 'r-', 'LineWidth', 2); % 画出拟合的pdf
title('北向一维轨迹差值GMM拟合');

figure(6);
x = linspace(min(err(:, 3)), max(err(:, 3)), 1000)';
histogram(err(:, 3), 'Normalization', 'pdf'); % 画出原数据概率直方图
hold on;
plot(x, pdf(gmm3, x), 'r-', 'LineWidth', 2); % 画出拟合的pdf
title('天向一维轨迹差值GMM拟合');
end

%% 坐标转换
function [x, y, z] = blh2xyz(B, L, H)
    % WGS-84椭球参数
    a = 6378137.0;          % 长半轴
    b = 6356752.3141;       % 短半轴
    e2 = 1 - (b/a)^2;      % 第一偏心率平方
    
    % 转换为弧度
    B_rad = deg2rad(B);
    L_rad = deg2rad(L);
    
    % 计算卯酉圈曲率半径
    N = a / sqrt(1 - e2 * sin(B_rad).^2);
    
    % XYZ坐标计算
    x = (N + H) .* cos(B_rad) .* cos(L_rad);
    y = (N + H) .* cos(B_rad) .* sin(L_rad);
    z = (N * (1 - e2) + H) .* sin(B_rad);
end

function [B, L, H] = xyz2blh(x, y, z)
    % WGS-84参数
    a = 6378137.0;
    b = 6356752.3141;
    e2 = 1 - (b/a)^2;
    
    % 计算经度L
    L = atan2(y, x);
    
    % 初始纬度B的估计
    p = sqrt(x.^2 + y.^2);
    B_prev = atan(z ./ p); % 初始值
    
    % 迭代求解纬度B和高度H
    max_iter = 100;
    tolerance = 1e-10;
    for i = 1:max_iter
        N = a ./ sqrt(1 - e2 * sin(B_prev).^2);
        H = p ./ cos(B_prev) - N;
        B_new = atan(z ./ (p - e2 * N .* sin(B_prev)));
        
        if max(abs(B_new - B_prev)) < tolerance
            break;
        end
        B_prev = B_new;
    end
    
    B = rad2deg(B_new);
    L = rad2deg(L);
end
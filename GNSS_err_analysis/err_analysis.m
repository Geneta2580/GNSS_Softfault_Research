%% 卡方误差绘图
clc
clear all
close all

status = 1; % 控制是否画出正常的残差图
file_err = 'GNSS_Last_ERR.csv'; % GNSS_Last_ERR GNSS_ERR
file_normal = 'GNSS_Last_ERR_normal.csv'; % GNSS_Last_ERR_normal.csv

%% 读取数据
% 带有偏差的残差
file_err_raw = readtable(file_err, 'Delimiter', ' ', 'ReadVariableNames', false);
err_param = table2array(file_err_raw); % 转换为数值矩阵

time_stamps_err = err_param(:, 1); % error^2，残差平方
chi2_err = err_param(:,  2);

% 不带偏移的残差
file_normal_raw = readtable(file_normal, 'Delimiter', ' ', 'ReadVariableNames', false);
normal_param = table2array(file_normal_raw); % 转换为数值矩阵

time_stamps_normal = normal_param(:, 1); % error^2，残差平方
chi2_normal = normal_param(:,  2);

%% 处理数据
% 7.815为检测阈值
threshold = 7.815; 
threshold_plot = threshold * ones(length(chi2_normal));

% 滑动平均
windowSize = 20;  
% smoothed_err = movmean(chi2_err, windowSize);
% smoothed_normal = movmean(chi2_normal, windowSize);
err_norm = rescale(chi2_err, 0, 1);
normal_norm = rescale(chi2_normal, 0, 1);

% % 剔除 chi2 中大于7.815的值，并同步处理对应的时间戳
% valid_indices = smoothed_err >= threshold;
% smoothed_err(valid_indices) = 0;
% chi2(valid_indices);
% time_stamps = time_stamps(valid_indices);

% 绘图
figure(1);
plot(time_stamps_err, err_norm);
hold on;
if(status)
    plot(time_stamps_normal, normal_norm); %#ok<UNRCH>
end
ylim([0 10]);
legend('加入偏移数据','正常数据');

% plot(time_stamps, threshold_plot, 'r'); %  , LineWidth=3
% xlim([time_stamps(1, 1),  time_stamps(end, 1)]);
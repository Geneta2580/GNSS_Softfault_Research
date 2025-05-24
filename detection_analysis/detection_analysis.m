clc
clear all
close all

%% 处理检测结果并绘图
% 初始化存储数组
filePath = 'detection.txt';
data = [];
fileID = fopen(filePath, 'r', 'n', 'utf8'); % 处理中文路径[1](@ref)

% 逐行读取并筛选
while ~feof(fileID)
    line = fgetl(fileID);
    if startsWith(line, 'data: ') % 判断行首是否为"data: "
        values = sscanf(line(7:end), '[%f, %f]');  % 从第7个字符开始解析
        if ~isempty(values)
            data = [data; values'];  % 按行存储
        end
    end
end
fclose(fileID);

time_stamps = data(:, 1);
detection = data(:, 2);

plot(time_stamps, detection);
hold on;

%% 处理误差卡方统计量
file_err = 'GNSS_Last_ERR.csv'; % GNSS_Last_ERR GNSS_ERR
file_err_raw = readtable(file_err, 'Delimiter', ' ', 'ReadVariableNames', false);
err_param = table2array(file_err_raw); % 转换为数值矩阵

time_stamps_err = err_param(:, 1); % error^2，残差平方
chi2_err = err_param(:,  2);

plot(time_stamps_err, chi2_err);
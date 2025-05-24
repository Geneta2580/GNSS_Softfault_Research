clc
clear all
close all

% t = 0:0.1:1000;
% f = zeros(1, size(t, 2));
% for i = 1:size(t, 2)
%     if(t(1, i) < 7.815)
%         f(1, i) = 1;
%     else
%         f(1, i) =  2 ./ (1 + exp(0.01*(t(1, i) - 7.815)));
%     end
% end
% 
% plot(t, f);

% 初始化存储数组
filePath = 'gnss_err_topic.txt';
dataValues = [];
fileID = fopen(filePath, 'r', 'n', 'utf8'); % 处理中文路径[1](@ref)

% 逐行读取并筛选
while ~feof(fileID)
    line = fgetl(fileID);
    if startsWith(line, 'data: ') % 判断行首是否为"data: "
        valueStr = strsplit(line, ' '); % 按空格分割字符串
        dataValues = [dataValues; str2double(valueStr{2})];
    end
end
fclose(fileID);

plot(dataValues);
hold on;

file_err = 'GNSS_Last_ERR.csv'; % GNSS_Last_ERR GNSS_ERR
file_err_raw = readtable(file_err, 'Delimiter', ' ', 'ReadVariableNames', false);
err_param = table2array(file_err_raw); % 转换为数值矩阵

time_stamps_err = err_param(:, 1); % error^2，残差平方
chi2_err = err_param(:,  2);

plot(time_stamps_err, chi2_err);
clc
clear all
close all

% 设置输入输出路径
inputFolder = 'input_data';
outputRoot = 'dataset';

% 获取所有符合命名规则的CSV文件（示例：1.0_800_500_s.csv）
fileList = dir(fullfile(inputFolder, '*_*_*_s.csv')); 

% 全局参数设置（滑动窗口相关参数保持不变）
windowSize = 50;        
step = 1;               
startID = 1;         
labelFile = 'label.csv'; 

% 初始化标签记录文件（带表头）
if ~exist(fullfile(outputRoot, labelFile), 'file')
    writecell({'value,label'}, fullfile(outputRoot, labelFile));
end


% 批量处理主循环
for fileIdx = 1:length(fileList)
    currentFile = fullfile(fileList(fileIdx).folder, fileList(fileIdx).name);
    [~, baseName] = fileparts(currentFile);
    
    % 从文件名提取参数（基于网页8的字符串分割）
    params = split(baseName, '_');  % 分解为[err_vel, err_start_time, err_during_time, err_kind]
    err_vel = str2double(params{1});
    err_start_time = str2double(params{2});
    err_during_time = str2double(params{3});

    file_err_raw = readtable(currentFile, 'Delimiter', ' ', 'ReadVariableNames', false);
    err_param = table2array(file_err_raw); % 转换为数值矩阵
    labels = zeros(size(err_param, 1), 1);

    % 进行标签标注
    err_timestamps = err_param(:, 1);
    fist_timestamps = err_param(1, 1);
    err_start_timestamps = fist_timestamps + err_start_time;
    err_end_timestamps = err_start_timestamps + err_during_time;
    
    [~, id_start] = min(abs(err_timestamps(:) - err_start_timestamps));
    [~, id_end] = min(abs(err_timestamps(:) - err_end_timestamps));
    labels(id_start:id_end, 1) = 1;
    err_param_new = [err_param(:, 2), labels];

    %% 滑动窗口时间切片
    totalSamples = size(err_param_new, 1);  % 总样本数
    numWindows = totalSamples - windowSize + 1; % 窗口总数
    
    for i = 1:step:numWindows
        % 提取当前窗口数据（包含标签列）
        currentWindow = err_param_new(i:i+windowSize-1, :);
    
        % 分离特征和标签
        features = currentWindow(:, 1:end-1); % 前N-1列为特征
        labels = currentWindow(:, end);        % 最后一列为标签
    
        % 生成唯一文件名（递增ID）
        filename = sprintf('%d.csv', startID);
        fullPath = fullfile(outputRoot, filename);
    
        % 保存窗口数据到CSV（不包含标签列）
        writematrix(features, fullPath);       % 若需保留标签列，替换为 `writematrix(currentWindow, fullPath)`
    
        % 记录标签文件
        fid = fopen(fullfile(outputRoot, labelFile), 'a', 'n', 'UTF-8'); 
        % 检测标签列是否包含1
        if any(labels == 1)
            fprintf(fid, '%s,1\n', filename); % 记录到标签文件
        else
            fprintf(fid, '%s,0\n', filename); % 将0记录到标签文件
        end
        fclose(fid); % 关闭文件

        startID = startID + 1; % ID递增
    end

end
%% GPS欺骗数据绘图
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

%% GPS数据转换并绘图
input1 = 'gnss_true_data_urban38.txt';
traj1 = GPS_transform(input1);
GPS_plot(traj1);

input2 = 'gnss_data.csv';
traj2 = GPS_transform(input2);
GPS_plot(traj2);

GPS_error_plot(traj2 - traj1);

%% GPS转换到ENU坐标系
function [traj] = GPS_transform(input)
global RA;
global E2;
global D2R;
% 读取整个文件
T = readtable(input,  'Delimiter',  ',',  'ReadVariableNames',  false); % gps_output.txt gnss_data.csv
% 删除第4列（文本列）
T(:, 4) = [];
navData = table2array(T); % 转换为数值矩阵
ref_pos = navData(1,  6:8); % 参考点[纬度,  经度,  高度](单位：度)

% 预处理
lat = navData(:,  6) * D2R;    % 纬度转弧度
lon = navData(:,  7) * D2R;    % 经度转弧度
hgt = navData(:,  8);          % 高程(m)

% 计算局部坐标
nPoints = size(navData,  1);
traj = zeros(nPoints,  3);

for i = 1:nPoints
    % 计算曲率半径[3](@ref)
    sinLat = sin(lat(i));
    Rm = RA*(1-E2) / (1 - E2*sinLat^2)^1.5;
    Rn = RA / sqrt(1 - E2*sinLat^2);

    % 坐标差计算
    dLat = lat(i) - ref_pos(1)*D2R; % 
    dLon = lon(i) - ref_pos(2)*D2R; % 

    % ENU坐标转换[5](@ref)
    traj(i, 1) = (Rn + hgt(i)) * dLon * cos(lat(i)); % 东向
    traj(i, 2) = (Rm + hgt(i)) * dLat;               % 北向
    traj(i, 3) = hgt(i) - ref_pos(3);                % 天向  
end
end

%% 绘图三维轨迹
function [] = GPS_plot(traj)
figure(1);
plot3(traj(:, 1),  traj(:, 2),  traj(:, 3),  'b-',  'LineWidth',  1.5);
xlabel('East (m)'); 
ylabel('North (m)'); 
zlabel('Up (m)');
title('三维轨迹图');
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
grid on;
hold on;

% 北向变化
subplot(3, 1, 2);
plot(traj(:, 2));
title('北向位移变化');
xlabel('时间/索引'); 
ylabel('位移(m)');
grid on;
hold on;

% 天向变化
subplot(3, 1, 3);
plot(traj(:, 3));
title('天向位移变化');
xlabel('时间/索引'); 
ylabel('位移(m)');
grid on;
hold on;

end

%% 绘图GPS误差数据
% 三个方向的数据
function [] = GPS_error_plot(traj)
figure(3);
% 东向变化
subplot(3,  1,  1);
plot(traj(:, 1));
title('东向位移误差变化');
xlabel('时间/索引'); 
ylabel('位移(m)');
grid on;
hold on;

% 北向变化
subplot(3, 1, 2);
plot(traj(:, 2));
title('北向位移误差变化');
xlabel('时间/索引'); 
ylabel('位移(m)');
grid on;
hold on;

% 天向变化
subplot(3, 1, 3);
plot(traj(:, 3));
title('天向位移误差变化');
xlabel('时间/索引'); 
ylabel('位移(m)');
grid on;
hold on;
end

%% 引入轨迹数据与GPS数据作比较
% raw = readtable('truth_traj.csv',  'Delimiter',  ' ',  'ReadVariableNames',  false);
% trj = table2array(raw); % 转换为数值矩阵
% 
% figure(3);
% plot3(trj(:, 2),  trj(:, 3),  trj(:, 4),  'b-',  'LineWidth',  1.5);
% xlabel('East (m)'); 
% ylabel('North (m)'); 
% zlabel('Up (m)');
% title('三维轨迹图');
% grid on;
% axis equal;
% hold on;

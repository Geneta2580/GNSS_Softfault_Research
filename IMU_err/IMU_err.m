clc
clear all
close all

file_err = 'IMU_ERR.csv'; % GNSS_Last_ERR GNSS_ERR
file_normal = 'IMU_ERR4.csv';

%% 读取数据
% 带有偏差的残差
file_err_raw = readtable(file_err, 'Delimiter', ' ', 'ReadVariableNames', false);
err_param = table2array(file_err_raw); % 转换为数值矩阵

time_stamps_err = err_param(:, 1); % error^2，残差平方
imu_gx_err = err_param(:,  2);
imu_gy_err = err_param(:,  3);
imu_gz_err = err_param(:,  4);
imu_ax_err = err_param(:,  5);
imu_ay_err = err_param(:,  6);
imu_az_err = err_param(:,  7);


% 不带偏移的残差
file_normal_raw = readtable(file_normal, 'Delimiter', ' ', 'ReadVariableNames', false);
normal_param = table2array(file_normal_raw); % 转换为数值矩阵

time_stamps_normal = normal_param(:, 1); % error^2，残差平方
imu_gx_normal = normal_param(:,  2);
imu_gy_normal = normal_param(:,  3);
imu_gz_normal = normal_param(:,  4);
imu_ax_normal = normal_param(:,  5);
imu_ay_normal = normal_param(:,  6);
imu_az_normal = normal_param(:,  7);

figure(1);
subplot(3, 1, 1);
plot(time_stamps_err, imu_gx_err);
hold on;
plot(time_stamps_normal, imu_gx_normal);

subplot(3, 1, 2);
plot(time_stamps_err, imu_gy_err);
hold on;
plot(time_stamps_normal, imu_gy_normal);

subplot(3, 1, 3);
plot(time_stamps_err, imu_gz_err);
hold on;
plot(time_stamps_normal, imu_gz_normal);

figure(2)
subplot(3, 1, 1);
plot(time_stamps_err, imu_ax_err);
hold on;
plot(time_stamps_normal, imu_ax_normal);

subplot(3, 1, 2);
plot(time_stamps_err, imu_ay_err);
hold on;
plot(time_stamps_normal, imu_ay_normal);

subplot(3, 1, 3);
plot(time_stamps_err, imu_az_err);
hold on;
plot(time_stamps_normal, imu_az_normal);

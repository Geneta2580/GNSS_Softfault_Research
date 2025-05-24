data = [(1:length(traj1(:, 1)))', traj1(:, 1)]; % 转换为二维坐标
origin = data(1, 1); % 取第一个点作为旋转原点
translated_data = data - origin; % 平移坐标系[1](@ref)

theta = 30; % 旋转角度（逆时针）
rotated_result = rotateAroundOrigin(data, origin, theta);

% 可视化对比
figure;
plot(data(:,1), data(:,2), 'bo-', 'DisplayName', '原始数据');
hold on;
plot(rotated_result(:,1), rotated_result(:,2), 'rs--', 'DisplayName', '旋转后');
scatter(origin(1), origin(2), 100, 'g', 'filled', 'DisplayName', '旋转原点');
legend;
axis equal;

function rotated_data = rotateAroundOrigin(data, origin, theta_deg)
    theta = deg2rad(theta_deg); % 角度转弧度
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)]; % 旋转矩阵[7](@ref)
    
    % 平移坐标系
    translated_data = data - origin;
    
    % 应用旋转变换
    rotated_translated = (R * translated_data')';
    
    % 保持以原点为中心的坐标系（不还原平移）
    rotated_data = rotated_translated; 
    
    % 若需要还原到原始坐标系，则执行：
    % rotated_data = rotated_translated + origin;
end

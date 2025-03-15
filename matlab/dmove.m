% 机械臂微分运动学控制代码
clc;
clear;
% 机械臂参数
L1 = 1; % 第一段臂长
L2 = 1; % 第二段臂长
L3 = 1; % 第三段臂长
L4 = 1; % 第四段臂长

% 关节速度限制

vlimit = [1,1,1,1];

% PID控制器参数
Kp = 1;
Ki = 0;
Kd = 0;

% 初始关节角度
theta = [0; 0; 0; 0];

% 目标末端执行器位置和姿态
x_desired = 1.5;
y_desired = 1.5;
z_desired = 0.5;
roll_desired = 0;
yaw_desired = 0;
pitch_desired = 0;

% 初始误差积分
integral_error = [0; 0; 0];
prev_error = [0; 0; 0];

% 时间步长和仿真时间
dt = 0.01;
t_end = 10;
t = 0:dt:t_end;

% 初始化存储变量
x_traj = zeros(length(t), 1);
y_traj = zeros(length(t), 1);
z_traj = zeros(length(t), 1);
theta_traj = zeros(length(t), 4);

% 主循环
for i = 1:length(t)
    % 当前末端执行器位置
    x_current = L1*cos(theta(1)) + L2*cos(theta(1)+theta(2)) + (L3+ L4*cos(theta(4)))*cos(theta(1)+theta(2)+theta(3));
    y_current = L1*sin(theta(1)) + L2*sin(theta(1)+theta(2)) + (L3+ L4*cos(theta(4)))*sin(theta(1)+theta(2)+theta(3));
    z_current = L4*sin(theta(4)); 
    
    % 计算误差
    error = [x_desired - x_current; y_desired - y_current; z_desired - z_current];
    
    % PID控制
    integral_error = integral_error + error * dt;
    derivative_error = (error - prev_error) / dt;
    prev_error = error;
    
    % 计算期望速度
    v_desired = Kp * error + Ki * integral_error + Kd * derivative_error;
    
    % 计算雅可比矩阵
    J = [-L1*sin(theta(1)) - L2*sin(theta(1)+theta(2)) - L3*sin(theta(1)+theta(2)+theta(3)), ...
         -L2*sin(theta(1)+theta(2)) - L3*sin(theta(1)+theta(2)+theta(3)), ...
         -(L3+ L4*cos(theta(4)))*sin(theta(1)+theta(2)+theta(3)),...
         -L4*sin(theta(4))*cos(theta(1)+theta(2)+theta(3));
         L1*cos(theta(1)) + L2*cos(theta(1)+theta(2)) + L3*cos(theta(1)+theta(2)+theta(3)), ...
         L2*cos(theta(1)+theta(2)) + L3*cos(theta(1)+theta(2)+theta(3)), ...
         (L3+ L4*cos(theta(4)))*cos(theta(1)+theta(2)+theta(3)),...
         -L4*sin(theta(4))*sin(theta(1)+theta(2)+theta(3));
         0, 0, 0, L4*cos(theta(4))];
    
    % 计算关节速度
    theta_dot = pinv(J) * v_desired;

    for j=1:4
        theta_dot(j) = max(min(theta_dot(j),vlimit(j)),-vlimit(j));
    end
    
    % 更新关节角度
    theta = theta + theta_dot * dt;
    
    % 存储轨迹
    x_traj(i) = x_current;
    y_traj(i) = y_current;
    z_traj(i) = z_current;
    theta_traj(i, :) = theta;
end

% 可视化
figure;
plot3(x_traj, y_traj, z_traj, 'b', 'LineWidth', 2);
hold on;
plot3(x_desired, y_desired, z_desired, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('机械臂末端执行器轨迹');
grid on;
legend('轨迹', '目标位置');

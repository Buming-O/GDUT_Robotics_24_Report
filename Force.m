clc;
clear all;
close all;

% 定义物体参数
mass = 3; % 物体质量，单位kg
g = 9.81; % 重力加速度，单位m/s^2
weight = mass * g; % 物体的重量

% 定义球的半径和中心位置
ball_radius = 40;
ball_center = [0, 0, -60];

% 定义接触点（假设已知）
cp1 = [38.6918  , 10.1462  ,-60];
cp2 = [-26.8920 ,  29.6111 , -60];
cp3 = [-13.8146  ,-37.5387 , -60];
 
% 初始化接触力方向（随机初始值）
f1_dir = rand(1, 3);
f2_dir = rand(1, 3);
f3_dir = rand(1, 3);

% 单位化方向向量
f1_dir = f1_dir / norm(f1_dir);
f2_dir = f2_dir / norm(f2_dir);
f3_dir = f3_dir / norm(f3_dir);

% 优化目标函数
fun = @(f_dir) objective_function(f_dir, cp1, cp2, cp3, ball_center, weight);

% 初始值
f_dir_init = [f1_dir, f2_dir, f3_dir];

% 优化选项
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');

% 优化约束条
A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];

% 运行优化
[f_dir_opt, fval] = fmincon(fun, f_dir_init, A, b, Aeq, beq, lb, ub, [], options);

% 提取优化后的力方向
f1_dir_opt = f_dir_opt(1:3);
f2_dir_opt = f_dir_opt(4:6);
f3_dir_opt = f_dir_opt(7:9);

% 计算接触力的大小
force_magnitude = weight / 3;

% 计算每个接触点的力
f1 = force_magnitude * f1_dir_opt / norm(f1_dir_opt);
f2 = force_magnitude * f2_dir_opt / norm(f2_dir_opt);
f3 = force_magnitude * f3_dir_opt / norm(f3_dir_opt);

% 计算每个接触点相对于物体重心的力矩
r1 = cp1 - ball_center;
r2 = cp2 - ball_center;
r3 = cp3 - ball_center;

% 计算总力矩
total_torque = cross(r1, f1) + cross(r2, f2) + cross(r3, f3);
% 检查总力矩是否为零
if norm(total_torque) < 1e-4
    disp('力矩平衡，抓取稳定');
else
    disp('力矩不平衡，抓取不稳定');
end
disp(norm(total_torque))

% 检查总力是否为零
total_force = f1 + f2 + f3 - [0,0, weight];
if norm(total_force) < 1
    disp('力平衡，抓取稳定');
else
    disp('力不平衡，抓取不稳定');
end
disp(norm(total_force))

% disp('接触点1的力：');
% disp(f1);
% disp('接触点2的力：');
% disp(f2);
% disp('接触点3的力：');
% disp(f3);

disp('总力矩：');
disp(total_torque);
disp('总力：');
disp(total_force);

% 目标函数定义
function cost = objective_function(f_dir, cp1, cp2, cp3, ball_center, weight)
    % 提取各个接触点的方向向量
    f1_dir = f_dir(1:3);
    f2_dir = f_dir(4:6);
    f3_dir = f_dir(7:9);

    % 单位化方向向量
    f1_dir = f1_dir / norm(f1_dir);
    f2_dir = f2_dir / norm(f2_dir);
    f3_dir = f3_dir / norm(f3_dir);

    % 计算接触力的大小
    force_magnitude = weight / 3;

    % 计算每个接触点的力
    f1 = force_magnitude * f1_dir;
    f2 = force_magnitude * f2_dir;
    f3 = force_magnitude * f3_dir;

    % 计算每个接触点相对于物体重心的力矩
    r1 = cp1 - ball_center;
    r2 = cp2 - ball_center;
    r3 = cp3 - ball_center;

    % 计算总力矩
    total_torque = cross(r1, f1) + cross(r2, f2) + cross(r3, f3);

    % 计算总力
    total_force = f1 + f2 + f3 - [0, 0, weight];

    % 目标函数值：力和力矩的不平衡量
    cost = norm(total_torque) + norm(total_force);
end

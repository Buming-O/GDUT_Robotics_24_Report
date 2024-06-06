function Planning_target_torque(best_cp1, best_cp2, best_cp3, ball_center, weight)
    % 生成初始猜测值
    f1_dir = rand(1, 3);
    f2_dir = rand(1, 3);
    f3_dir = rand(1, 3);
    
    % 单位化方向向量
    f1_dir = f1_dir / norm(f1_dir);
    f2_dir = f2_dir / norm(f2_dir);
    f3_dir = f3_dir / norm(f3_dir);
    f_dir_init = [f1_dir, f2_dir, f3_dir];

    % 目标函数
    fun = @(f_dir) objective_function(f_dir, best_cp1, best_cp2, best_cp3, ball_center, weight);

    % 优化设置
    options = optimoptions('fmincon', 'Display', 'notify-detailed', 'Algorithm', 'interior-point');
    
    % 运行优化
    [f_dir_opt, fval] = fmincon(fun, f_dir_init, [], [], [], [], [], [], [], options);

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
    r1 = best_cp1 - ball_center;
    r2 = best_cp2 - ball_center;
    r3 = best_cp3 - ball_center;

    % 计算总力矩与总力
    total_torque = cross(r1, f1) + cross(r2, f2) + cross(r3, f3);
    total_force = f1 + f2 + f3 - [0, 0, weight];
    
    % 使用 sprintf 将 norm_torque 转换为纯数字格式
    formatted_torque = sprintf('%.10f', norm(total_torque));
    
    % 确保所有力和力矩的和小于容忍值
    if norm(total_torque) < 1e-4 && norm(total_force) < 5e-1
        disp([formatted_torque, 'N/m, ', num2str(norm(total_force)), 'N, 力矩与力平衡，抓取稳定']);
    else
        disp('力矩或力不平衡，抓取不稳定');
    end

    % 输出接触点的力和总力矩、总力
    fprintf('接触点1的力：%.4f, %.4f, %.4f\n', f1);
    fprintf('接触点2的力：%.4f, %.4f, %.4f\n', f2);
    fprintf('接触点3的力：%.4f, %.4f, %.4f\n', f3);
    fprintf('总力矩：%.4f, %.4f, %.4f\n', total_torque);
    fprintf('总力：%.4f, %.4f, %.4f\n', total_force);
end

function fval = objective_function(f_dir, best_cp1, best_cp2, best_cp3, ball_center, weight)
    % 提取每个接触点的力方向
    f1_dir = f_dir(1:3);
    f2_dir = f_dir(4:6);
    f3_dir = f_dir(7:9);
    
    % 计算接触力的大小
    force_magnitude = weight / 3;

    % 计算每个接触点的力
    f1 = force_magnitude * f1_dir / norm(f1_dir);
    f2 = force_magnitude * f2_dir / norm(f2_dir);
    f3 = force_magnitude * f3_dir / norm(f3_dir);

    % 计算每个接触点相对于物体重心的力矩
    r1 = best_cp1 - ball_center;
    r2 = best_cp2 - ball_center;
    r3 = best_cp3 - ball_center;

    % 计算总力矩与总力
    total_torque = cross(r1, f1) + cross(r2, f2) + cross(r3, f3);
    total_force = f1 + f2 + f3 - [0, 0, weight];

    % 定义目标函数值（目标是最小化力矩和力的范数）
    fval = norm(total_torque) + norm(total_force);
end

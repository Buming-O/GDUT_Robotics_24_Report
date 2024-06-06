% 定义检查工作空间限制的函数
function in_limits = check_workspace_limits(point, workspace_limits)
    in_limits = all(point >= workspace_limits(:, 1)') && all(point <= workspace_limits(:, 2)');
end
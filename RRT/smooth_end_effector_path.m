% 定义平滑路径函数
function smoothed_path = smooth_end_effector_path(path)
    num_points = size(path, 1);
    smoothed_path = path;
    for i = 2:num_points-1
        smoothed_path(i, :) = (path(i-1, :) + path(i+1, :)) / 2;
    end
end
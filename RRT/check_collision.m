% 定义检查碰撞的函数
function in_collision = check_collision(point, obstacle_center, obstacle_radius)
    in_collision = norm(point - obstacle_center) < obstacle_radius+3;
end
% 定义生成新节点的函数
function q_new = gen_node(q_nearest, q_rand, step_size)
    direction = q_rand - q_nearest;
    distance = norm(direction);
    q_new = q_nearest + (direction / distance) * min(step_size, distance);
end
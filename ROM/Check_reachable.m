% 加载手指的工作空间点
load('workspace_points1.mat', 'workspace_points1');
load('workspace_points2.mat', 'workspace_points2');
load('workspace_points3.mat', 'workspace_points3');

% 定义球的半径
ball_radius = 40;

% 定义网格范围和分辨率
x_range = -100:5:100;
y_range = -100:5:100;
z_range = -80:2:0-ball_radius;
% 初始化可达空间集合
reachable_points = [];
%%
% profile on; % 开启性能分析
% 遍历网格中的每一个位置
for x = x_range
    for y = y_range
        for z = z_range
            ball_center = [x, y, z];

            % 计算到每个手指工作空间的距离
            dist_to_workspace1 = pdist2(workspace_points1, ball_center);
            dist_to_workspace2 = pdist2(workspace_points2, ball_center);
            dist_to_workspace3 = pdist2(workspace_points3, ball_center);

            % 判断是否在所有手指的工作空间内
            is_reachable1 = any(dist_to_workspace1 < ball_radius);
            is_reachable2 = any(dist_to_workspace2 < ball_radius);
            is_reachable3 = any(dist_to_workspace3 < ball_radius);

            if is_reachable1 && is_reachable2 && is_reachable3
                reachable_points = [reachable_points; ball_center];
            end
        end
    end
end
% profile off; % 关闭性能分析
% profile viewer; % 显示性能分析结果
%%
% 绘制可达空间
figure;
scatter3(reachable_points(:,1), reachable_points(:,2), reachable_points(:,3), 'filled');
hold on;

% 绘制原始工作空间点（可选）
scatter3(workspace_points1(:,1), workspace_points1(:,2), workspace_points1(:,3), 'r.');
scatter3(workspace_points2(:,1), workspace_points2(:,2), workspace_points2(:,3), 'g.');
scatter3(workspace_points3(:,1), workspace_points3(:,2), workspace_points3(:,3), 'b.');

xlabel('X');
ylabel('Y');
zlabel('Z');
title('球的可达空间');
grid on;
hold off;

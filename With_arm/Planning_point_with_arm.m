function [best_cp1, best_cp2, best_cp3] = Planning_point_with_arm(ball_center, ball_radius, num_points, angle_threshold, distance_threshold, shp1, shp2, shp3)
    % 在球的表面上生成候选接触点
    theta = linspace(0, 2*pi, num_points);
    phi = linspace(pi/2, pi, num_points);
    [Theta, Phi] = meshgrid(theta, phi);
    X = ball_radius * sin(Phi) .* cos(Theta) + ball_center(1);
    Y = ball_radius * sin(Phi) .* sin(Theta) + ball_center(2);
    Z = ball_radius * cos(Phi) + ball_center(3);
    candidate_points = [X(:), Y(:), Z(:)];
    
    % 划分候选接触点到三个象限
    theta_deg = rad2deg(Theta(:));
    points_sector1 = candidate_points(theta_deg >= 0-angle_threshold & theta_deg < 0+angle_threshold, :);
    points_sector2 = candidate_points(theta_deg >= 120-angle_threshold & theta_deg < 120+angle_threshold, :);
    points_sector3 = candidate_points(theta_deg >= 240-angle_threshold & theta_deg < 240+angle_threshold, :);

    % 初始化可达的接触点
    reachable_points1 = points_sector1;
    reachable_points2 = points_sector2;
    reachable_points3 = points_sector3;

    % 确保形成稳定的抓取三角形
    if ~isempty(reachable_points1) && ~isempty(reachable_points2) && ~isempty(reachable_points3)
        % 初始化最小距离和对应的接触点
        min_distance = inf;
        best_cp1 = [];
        best_cp2 = [];
        best_cp3 = [];
        
        % 遍历所有可能的接触点组合
        for i = 1:size(reachable_points1, 1)
            for j = 1:size(reachable_points2, 1)
                for k = 1:size(reachable_points3, 1)
                    cp1 = reachable_points1(i, :);
                    cp2 = reachable_points2(j, :);
                    cp3 = reachable_points3(k, :);

                    % 检查三个点是否共线
                    if ~iscollinear(cp1, cp2, cp3)
                        % 计算接触点的质心
                        centroid = (cp1 + cp2 + cp3) / 3;

                        % 计算质心与球心之间的距离
                        distance = norm(centroid - ball_center);

                        % 更新最小距离和最佳接触点
                        if distance < min_distance
                            min_distance = distance;
                            best_cp1 = cp1;
                            best_cp2 = cp2;
                            best_cp3 = cp3;
                        end
                    end
                end
            end
        end
        
        % 显示结果
        if min_distance < distance_threshold
            fprintf('质心与球心之间的距离:%.4f,形成稳定的抓取三角形\n', min_distance);
        else
            disp('抓取三角形不稳定');
        end
    else
        disp('没有足够的可达接触点来形成稳定的抓取');
    end

    plot3(candidate_points(:,1),candidate_points(:,2),candidate_points(:,3),'k.','MarkerSize',1);
    plot3(reachable_points1(:, 1), reachable_points1(:, 2), reachable_points1(:, 3), 'r.', 'MarkerSize', 5, 'LineWidth', 1);
    plot3(reachable_points2(:, 1), reachable_points2(:, 2), reachable_points2(:, 3), 'g.', 'MarkerSize', 5, 'LineWidth', 1);
    plot3(reachable_points3(:, 1), reachable_points3(:, 2), reachable_points3(:, 3), 'b.', 'MarkerSize', 5, 'LineWidth', 1);
    plot3(best_cp1(1), best_cp1(2), best_cp1(3), 'ro', 'MarkerSize', 10, 'LineWidth', 4);
    plot3(best_cp2(1), best_cp2(2), best_cp2(3), 'go', 'MarkerSize', 10, 'LineWidth', 4);
    plot3(best_cp3(1), best_cp3(2), best_cp3(3), 'bo', 'MarkerSize', 10, 'LineWidth', 4);
end
%%　
% 检查三个点是否共线
function result = iscollinear(p1, p2, p3)
    result = norm(cross(p2 - p1, p3 - p1)) < 1e-10;
end

function reachable_points = find_reachable_points_shp(points_sector, shp)
    % 使用alphaShape检查点是否在多边形体内
    reachable_points = [];
    for i = 1:size(points_sector, 1)
        contact_point = points_sector(i, :);
        if inShape(shp, contact_point)
            reachable_points = [reachable_points; contact_point];
        end
    end
end

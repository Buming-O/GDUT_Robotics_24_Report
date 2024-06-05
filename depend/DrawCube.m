function DrawCube(center, side_length, col)
    % center: 正方体的中心 [x, y, z]
    % side_length: 正方体的边长
    % col: 正方体的颜色

    % 计算正方体的八个顶点坐标
    d = side_length / 2;
    vertices = [center(1)-d, center(2)-d, center(3)-d;
                center(1)+d, center(2)-d, center(3)-d;
                center(1)+d, center(2)+d, center(3)-d;
                center(1)-d, center(2)+d, center(3)-d;
                center(1)-d, center(2)-d, center(3)+d;
                center(1)+d, center(2)-d, center(3)+d;
                center(1)+d, center(2)+d, center(3)+d;
                center(1)-d, center(2)+d, center(3)+d];

    % 定义正方体的面
    faces = [1, 2, 3, 4;
             5, 6, 7, 8;
             1, 2, 6, 5;
             2, 3, 7, 6;
             3, 4, 8, 7;
             4, 1, 5, 8];

    % 绘制正方体
    patch('Vertices', vertices, 'Faces', faces, 'FaceColor', col);
%     axis equal;
%     xlabel('X');
%     ylabel('Y');
%     zlabel('Z');
%     grid on;
%     view(3); % 设置为三维视角
end

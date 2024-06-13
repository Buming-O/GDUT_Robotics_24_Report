
% 假设已有一个三维图形的 fig 文件
fig = openfig('111.fig');

% 获取当前轴
ax = gca;

% 创建一个视频对象
videoObj = VideoWriter('111.avi');
videoObj.FrameRate = 20; % 设置帧率
open(videoObj);

% 设置视角参数
az = 0;
el = 30;

% 旋转视角并保存每一帧到视频
for k = 1:50
    % 更新视角
    view(ax, az, el);
    az = az + 2; % 每次增加1度
    drawnow;
    axis equal;
    % Capture the plot as an image
    frame = getframe(gcf);
    % Write the frame to the video
    writeVideo(videoObj, frame);
end

for k = 1:25
    % 更新视角
    view(ax, az, el);
    el = el - 2; % 每次增加1度
    drawnow;
    axis equal;
    % Capture the plot as an image
    frame = getframe(gcf);
    % Write the frame to the video
    writeVideo(videoObj, frame);
end

% 关闭视频对象
close(videoObj);
disp('Video saved successfully');
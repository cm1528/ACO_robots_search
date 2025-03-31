clc;
clear;
close all;
% 参数设置
total = 3;            % 蚂蚁数量
iterations = 5000;    % 迭代次数
phera_up = 5;        % 信息素更新系数
rho = 0.8;            % 信息素衰减系数
alpha = 1;            % 信息素的重要性
beta = 2;             % 距离的重要性
grid_size = 10;      % 网格大小 (10x10)

% 初始化环境与障碍物
env = ones(grid_size, grid_size);  % 创建空的环境网格
goal = [10, 10];                   % 目标位置 (固定，不在障碍物中)
goal_radius = 0;                   % 目标半径 (目标范围)

% 定义障碍物位置和大小
x1_1 = 2; y1_1 = 2; x2_1 = 3; y2_1 = 3;  % 第一个障碍物：2x2
x1_2 = 5; y1_2 = 5; x2_2 = 7; y2_2 = 7;  % 第二个障碍物：2x2

% 标记环境中的障碍物
env(x1_1:x2_1, y1_1:y2_1) = 0;  % 设置第一个障碍物的区域为0
env(x1_2:x2_2, y1_2:y2_2) = 0;  % 设置第二个障碍物的区域为0

% 初始化蚂蚁的位置
ant_positions = zeros(total, 2);   % 存储蚂蚁的位置
ant_trajectories = cell(total, 1); % 存储每只蚂蚁的路径
ant_reached = zeros(total, 1);     % 每只蚂蚁是否到达目标的标志 (1 = 到达目标, 0 = 未到达)
found_goal = 0;                    % 标志变量，指示是否有蚂蚁找到目标

ant_positions(1, :) = [1, 9];
ant_positions(2, :) = [1, 2];
ant_positions(3, :) = [5, 5];

% 初始化信息素矩阵
phera = zeros(grid_size, grid_size); % 信息素矩阵

% 定义蚂蚁的移动方向（8个方向）
lm = [0 -1; 1 -1; 1 0; 1 1; 0 1; -1 1; -1 0; -1 -1];
li = 1:8; % 方向索引

% 探索参数
ant_explore_limit = 100;  % 最大步数（超过后重新探索）
explore_prob = 0.2;       % 随机探索的概率

% 仿真循环
figure;
hold on;
axis([0 grid_size 0 grid_size]); % 设置绘图范围
title('\fontname{宋体}蚁群算法分布式协同搜索');
xlabel('\fontsize{10}\fontname{Times New Roman}X\fontname{宋体}轴');
ylabel('\fontsize{10}\fontname{Times New Roman}Y\fontname{宋体}轴');
colors = ['r', 'g', 'b']; % 为不同的蚂蚁设置不同的颜色

% 主仿真循环
for iter = 1:iterations
    for j = 1:total
        % 如果蚂蚁已经到达目标，跳过当前蚂蚁
        if ant_reached(j) == 1
            continue;
        end
        r_temp = ant_positions(j, 1); % 当前行
        c_temp = ant_positions(j, 2); % 当前列
        best_move = [-1, -1];          % 该蚂蚁的最佳移动方向
        min_pheromone = -inf;

        % 检查蚂蚁是否已接近目标
        if abs(r_temp - goal(1)) <= goal_radius && abs(c_temp - goal(2)) <= goal_radius
            ant_reached(j) = 1; % 标记蚂蚁已到达目标
            found_goal = 1;     % 设置目标已找到标志
            
            % 在目标处留下信息素
            phera(r_temp, c_temp) = phera(r_temp, c_temp) + phera_up;
            break; % 蚂蚁完成搜索，退出当前循环
        end

        % 探索8个可能的方向
        for k = 1:8
            r = r_temp + lm(k, 1); 
            c = c_temp + lm(k, 2);

            % 检查移动是否在网格范围内且不进入障碍物
            if r > 0 && c > 0 && r <= grid_size && c <= grid_size && env(r, c) == 1
                % 计算信息素影响和与目标的距离
                pheromone = phera(r, c)^alpha;
                distance = ((goal(1) - r)^2 + (goal(2) - c)^2 + 1)^(-beta);
                trans = pheromone * distance;

                % 根据信息素选择最佳移动方向
                if trans > min_pheromone
                    min_pheromone = trans;
                    best_move = [lm(k, 1), lm(k, 2)];
                end
            end
        end

        % 如果没有找到最佳移动（即被困住），强制随机探索
        if all(best_move == -1) || rand < explore_prob
            best_move = [randi([-1, 1]), randi([-1, 1])];  % 随机移动
        end

        % 更新蚂蚁的位置
        if best_move(1) ~= -1
            % 计算新的位置
            new_position = ant_positions(j, :) + best_move;

            % 确保新的位置在 1 到 10 的范围内
            new_position(1) = max(1, min(new_position(1), 10));  % 限制行坐标
            new_position(2) = max(1, min(new_position(2), 10));  % 限制列坐标
            
            % 更新蚂蚁位置
            if env(new_position(1), new_position(2)) == 1
                ant_positions(j, :) = new_position;
            else
                % 如果新位置在障碍物区域，则重新选择位置
                new_position = ant_positions(j, :) + best_move;  % 或重新生成随机位置
            end
            ant_trajectories{j} = [ant_trajectories{j}; ant_positions(j, :)]; % 添加新的位置到路径中
            
            if env(ant_positions(j, 1), ant_positions(j, 2)) == 1
                % 在新位置更新信息素
                phera(ant_positions(j, 1), ant_positions(j, 2)) = phera(ant_positions(j, 1), ant_positions(j, 2)) + phera_up;
            end
        end
    end

    % 检查是否所有蚂蚁都已经到达目标
    if all(ant_reached == 1)
        disp('所有蚂蚁都已到达目标，结束仿真。');
        break; % 所有蚂蚁找到目标，退出循环
    end
    
    % 衰减信息素
    phera = phera * rho;

    % 绘制环境
    imshow(env, 'InitialMagnification', 'fit');
    hold on;
    plot(goal(2), goal(1), 'kp', 'MarkerSize', 10, 'MarkerFaceColor', 'k'); % 目标标记

    % 绘制每只蚂蚁的路径
    for j = 1:total
        plot(ant_trajectories{j}(:, 2), ant_trajectories{j}(:, 1), colors(j), 'LineWidth', 2);
    end
    
    pause(0.1);
    drawnow;
end

% 显示蚂蚁的最终路径
title('\fontname{宋体}蚁群算法分布式协同搜索');
% 绘制图例
legend( '\fontname{宋体}目标','\fontname{宋体}机器人\fontname{Times New Roman}1', '\fontname{宋体}机器人\fontname{Times New Roman}2', '\fontname{宋体}机器人\fontname{Times New Roman}3');
hold off;

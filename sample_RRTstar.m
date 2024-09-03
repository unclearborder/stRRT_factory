function RRT_star
    % パラメータ設定
    max_iter = 1000;
    step_size = 1.0;
    goal_sample_rate = 0.1;
    search_radius = 2.0;

    % スタートとゴールの設定
    start_node = [2, 2];
    goal_node = [8, 8];

    % 障害物の定義（矩形で表現）
    obstacles = [3, 3, 2, 2; 
                 6, 6, 2, 2];

    % 初期ツリーの設定
    nodes = start_node;
    parents = [0]; % ノードの親を保持

    % 描画設定
    figure;
    hold on;
    axis([0 10 0 10]);
    rectangle('Position', [goal_node(1)-0.5, goal_node(2)-0.5, 1, 1], 'Curvature', [1, 1], 'EdgeColor', 'r');
    for i = 1:size(obstacles, 1)
        rectangle('Position', obstacles(i,:), 'FaceColor', [0 0 0]);
    end

    % RRT*の実行
    for iter = 1:max_iter
        % 新しいランダムノードを生成
        if rand() < goal_sample_rate
            rand_node = goal_node;
        else
            rand_node = [rand() * 10, rand() * 10];
        end
        
        % 既存のノードに最も近いノードを見つける
        [nearest_node, nearest_idx] = findNearestNode(nodes, rand_node);
        
        % 新しいノードをステップサイズに従って生成
        new_node = steer(nearest_node, rand_node, step_size);
        
        % 衝突検出
        if ~isCollision(nearest_node, new_node, obstacles)
            % 新しいノードをツリーに追加
            nodes = [nodes; new_node];
            parents = [parents; nearest_idx];
            
            % 新しいノードを最適化（再接続）
            nodes = optimizePath(nodes, parents, new_node, goal_node, search_radius, obstacles);
            
            % 新しいエッジを描画
            plot([nearest_node(1), new_node(1)], [nearest_node(2), new_node(2)], 'g');
            drawnow;
            
            % ゴールに近ければ終了
            if norm(new_node - goal_node) < step_size
                disp('Goal reached!');
                break;
            end
        end
    end
end

function [nearest_node, nearest_idx] = findNearestNode(nodes, rand_node)
    distances = sqrt(sum((nodes - rand_node).^2, 2));
    [~, nearest_idx] = min(distances);
    nearest_node = nodes(nearest_idx, :);
end

function new_node = steer(nearest_node, rand_node, step_size)
    direction = rand_node - nearest_node;
    distance = norm(direction);
    direction = direction / distance;
    new_node = nearest_node + step_size * direction;
end

function collision = isCollision(node1, node2, obstacles)
    for i = 1:size(obstacles, 1)
        obstacle = obstacles(i, :);
        if lineRectIntersect(node1, node2, obstacle)
            collision = true;
            return;
        end
    end
    collision = false;
end

function intersect = lineRectIntersect(p1, p2, rect)
    line_min = min(p1, p2);
    line_max = max(p1, p2);
    rect_min = rect(1:2);
    rect_max = rect(1:2) + rect(3:4);
    
    intersect = all(line_max >= rect_min) && all(line_min <= rect_max);
end

function nodes = optimizePath(nodes, parents, new_node, goal_node, radius, obstacles)
    for i = 1:size(nodes, 1)
        if norm(nodes(i, :) - new_node) < radius
            if ~isCollision(nodes(i, :), new_node, obstacles)
                parents(end) = i; % 親ノードの更新
                plot([nodes(i, 1), new_node(1)], [nodes(i, 2), new_node(2)], 'b'); % 再接続の描画
            end
        end
    end
end

function issue_flag = psuedo_obs_check_line_oct(x, node_jj, obstacle_edge, bound, Dc)
% Inputs of this function are
% x: The newly sampled point
% node_jj: The existing nodes information (See the beginning of "main.m for more information")
% obstacle_edge: The information of obstacles
% bound: This define the region of the enviroment
% Dc: 衝突回避半径

% This function detemines if the transition from the node_jj to x is collison-free or not
% 0 (=false):衝突無し
% 1 (=true): 衝突発生

% Initialize Issue_flag
issue_flag = false;
issue_boundary = true;

obs_edge = obstacle_edge([1 3; 2 3; 2 4; 1 4]);

% 新しい点と最近接ノードを接続した直線上に障害物が存在しないかチェック

% 既存のツリー上の点(最近接ノード)から新しい点に伸ばす．
x0 = node_jj.x(1:2);
xF = x(1:2);


% 障害物の直線の個数
num_obs_edge = length(obs_edge);

% reshaping the information of edges (start and end points)
obs_st = obs_edge([1 2 3 4],:);
obs_end = obs_edge([2 3 4 1],:);
% obs_st = reshape([obstacle_edge(:).start], [2, num_obs_edge]).';
% obs_end = reshape([obstacle_edge(:).end], [2, num_obs_edge]).';

% [obs_st1, obs_end1; obs_st2, obs_end2; ...]
obstacle_edge_list = [obs_st, obs_end];
obs_edge_delete_list = zeros(num_obs_edge, 1);

% 全ての障害物の直線でチェック
for k = 1:num_obs_edge
    x2_st = obs_st(k);
    x2_end = obs_end(k);
    
    % check direct collision here
    [min_dist, is_cross] = minDist_two_LineSeg_in(x0, xF, x2_st, x2_end);
    
    % 衝突発生 → 「2つの線分(既存のツリー上ノードと新しい点，障害物の直線)が交差
    %              or 2つの線分間最小距離がDc以下」
    if is_cross == true || min_dist < Dc
        issue_flag = true;
        return
    else
        obs_edge_delete_list(k) = k;
    end
end

% deletes edges for which the collision cannot happen
obstacle_edge_list( (obs_edge_delete_list ~= 0), : ) = [];
num_obs_edge = size(obstacle_edge_list, 1);

% Similar check for outer boundry 
boundary_X_dist = min( [( x0(1) - bound(1).x(1) ), ( xF(1) - bound(1).x(1) )...
    ( bound(1).x(2) - x0(1) ), ( bound(1).x(2) - xF(1) ) ] );
boundary_Y_dist = min( [( x0(2) - bound(2).x(1) ), ( xF(2) - bound(2).x(1) )...
    ( bound(2).x(2) - x0(2) ), ( bound(2).x(2) - xF(2) ) ] );


if boundary_X_dist > Dc && boundary_Y_dist > Dc
    issue_boundary = false;
elseif boundary_X_dist <= Dc || boundary_Y_dist <= Dc
    issue_flag = true;
    return
end


if num_obs_edge == 0 && issue_boundary == false
    issue_flag = false;
    return
end
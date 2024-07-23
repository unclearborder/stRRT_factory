function [parent, value, nbor_issue, plotobj_neighbors, theta, omega, v] = find_parent(x, node, radius, In_list_ID, nearest, theta_from_nearest, omega_from_nearest, v_from_nearest)

param = load('param.mat');
map = load('map.mat');
% obstacle_edge = param.obstacle_edge;
% other_ship    = param.other_ship;
Dc            = param.Dc;
bound         = param.bound;
% v_o           = param.v_o;
t_max         = param.t_max;
num_obs_edge_ini = length(map.AreaMap.name);

% 出力の定義＆初期値
nbor_issue = 0; parent = 0; value = 0; theta = 0; omega = 0; v = 0; plotobj_neighbors = plot3(0,0,0);

issue_flag1 = false(num_obs_edge_ini,1);

% 最近接ノードから新しい点を結んだ線が障害物と干渉していないかチェック
% issue_flag = 0: 問題なし
% issue_flag = 1: 衝突発生
for j = 1:num_obs_edge_ini
    if ~strcmp(map.AreaMap.type(j),'circle') == true
        obstacle_edge = [map.AreaMap.offset_start(j,:);
                         map.AreaMap.offset_finish(j,:)];
        issue_flag1(j) = psuedo_obs_check_line_oct( x, node(nearest), obstacle_edge, bound, Dc);
        
    elseif strcmp(map.AreaMap.type(j),'circle') == true
        obs_c = map.AreaMap.offset_center(j);
        obs_r = map.AreaMap.r(j);
        issue_flag1(j) =  psuedo_obs_check_circle_oct(x, node(nearest), obs_c, obs_r, Dc);
    end

end
% issue_flag1 = psuedo_obs_check_line_oct( x, node(nearest), obstacle_edge, bound, Dc);
% issue_flag2 = psuedo_obs_check_line_ship(x, node(nearest), other_ship, Dc, v_o, t_max);
% 
% issue_flag_nearest  = issue_flag1 || issue_flag2;
issue_flag_nearest  = ~isempty(issue_flag1(issue_flag1 == 1));

% 新ノードとnearestとの距離などを計算
if issue_flag_nearest == 0    
%     dim = numel(x);
%     x_nearest_vec = [node(nearest).x].';
%     x_nearest_mat = reshape( x_nearest_vec, [dim, numel(x)/dim]);
%     cost   = dist_ig_mat( x_nearest_mat, x.');
    cost   = calc_cost(node(nearest), x, theta_from_nearest, omega_from_nearest, v_from_nearest);
    theta  = theta_from_nearest;
    omega  = omega_from_nearest;
    v      = v_from_nearest;
    parent = nearest;
    value  = [node(nearest).value].' + cost.';
end

% nearest以外に近接ノードがないかチェックする．
In_list_ID_without_nearestID = find(In_list_ID ~= nearest); % In_list_IDからnearestのIDを削除したもの


if isempty(In_list_ID_without_nearestID) == 1 && issue_flag_nearest == 1 %nearestしか近接ノードが無くかつnearestも障害物と干渉するとき
   
    nbor_issue = 1; % 親ノードなしなのでサンプリングしなおす

elseif isempty(In_list_ID_without_nearestID) == 1 && issue_flag_nearest == 0  %nearestしか近接ノードが無くnearestで問題ないとき

    nbor_issue = 0;

else % nearest以外にも近接ノードがあるとき

    Is_only_neibor_ID = false; % falseで距離の計算も行う
    is_rewiring       = false; % rewiringのときかどうか

    [nbors_ID, dist_xk2node_i,plotobj_neighbors] = find_neighbors( x, node, radius, In_list_ID_without_nearestID, Is_only_neibor_ID, is_rewiring); 
  
    % 衝突＆ノンホロノミック拘束チェック
    num_nbor = numel(nbors_ID);
    issue_flag = true(num_nbor, 1);
    theta_tmp = [];
    omega_tmp = [];
    v_tmp     = [];
    
    for jj = 1:num_nbor
        node_jj = node(nbors_ID(jj));
        
        issue_flag1 = psuedo_obs_check_line_oct( x, node_jj, obstacle_edge, bound, Dc);   % 陸地との衝突チェック
%         issue_flag2 = psuedo_obs_check_line_ship(x, node_jj, other_ship, Dc, v_o, t_max); % 他船との衝突チェック
        % ノンホロノミックな拘束満たすかチェック
        [issue_flag3, theta_new, omega_new, v_new] = nonholonomic_check2(x, node_jj); % ①．宮崎先生の論文の方法
        %issue_flag3 = path_curvature_check(x, node_jj, node, curvature_threshold_in_radian);                % ②．経路の曲率(curvature)を考える．  
       
        issue_flag(jj) = issue_flag1 || issue_flag3;
        
        if issue_flag(jj) == 0
            theta_tmp(end+1) = theta_new;
            omega_tmp(end+1) = omega_new;
            v_tmp(end+1)     = v_new;

        end
        
    end

    % 問題無しの近接ノードIDを保存
    In_list_no_issue_ID = nbors_ID(~issue_flag);             

    if isempty(In_list_no_issue_ID) == false     
        % ここでコストの計算
        cost = [];
        for ii = 1:length(In_list_no_issue_ID)
           cost(end+1) = calc_cost(node(In_list_no_issue_ID(ii)), x, theta_tmp(ii), omega_tmp(ii), v_tmp(ii)); 
        end
            
        %val = [node(In_list_no_issue_ID).value].' + dist_xk2node_i(~issue_flag).';
        val = [node(In_list_no_issue_ID).value].' + cost';
        [val_min, k_min] = min(val);
        theta     = theta_tmp(k_min);
        omega     = omega_tmp(k_min);
        v         = v_tmp(k_min);
        parent_ID = In_list_no_issue_ID(k_min);
        parent = parent_ID;
        value = val_min;

    elseif isempty(In_list_no_issue_ID) == true && issue_flag_nearest == 1
        
        nbor_issue = 1;
        
    end

end


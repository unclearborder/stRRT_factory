function [ nearest, ed ] = find_nearest_2Djudge( node, In_list_ID, x)
% Find the nearest node from nodes in the tree (In_list_ID) to the node (x) and the minimum distance

param = load('param.mat');
v_max = param.v_max;
%allowable_angle = param.allowable_angle;
candidate_parent_node_ID = [];
    

    for ii = 1:length(In_list_ID)
        node_ID = In_list_ID(ii);
        
        time_at_nodejj   = node(node_ID).x(3);
        angle_difference = angle_difference_nodejj_and_x(node(node_ID),x);
    
%         if time_at_nodejj < x(3)
        if time_at_nodejj <= x(3)
            candidate_parent_node_ID(end+1) =  node_ID;
        end
    
    end
    
    
    if isempty(candidate_parent_node_ID) % もし親ノードの候補が見つからなかったとき、
    
        ed      = 0; % ed<radius_minにしてサンプルをやり直す。
        nearest = 0;
    
    else % 親ノードの候補が存在するとき、各候補と新ノード間の距離を計算
        candidate_parent_node = reshape([node(candidate_parent_node_ID).x], [3, numel(candidate_parent_node_ID)]).';
        d = sqrt( sum( ((candidate_parent_node(:,1:2) - x(1:2)).^2)'));
%         v_max_t = v_max*(x(3) - candidate_parent_node(:,3))';
        v_max_t = 100000;
        d_time_sufficient = d(d < v_max_t);
        if isempty(d_time_sufficient)
            ed      = 0;
            nearest = 0;
        else
            candidate_parent_node_ID_sufficient = candidate_parent_node_ID(d < v_max_t);
        % 最小距離，最小となる既存のノードの番号
            [ed, d_min_index] = min(d_time_sufficient);
            nearest = candidate_parent_node_ID_sufficient(d_min_index);
        end
    
    end

end

    



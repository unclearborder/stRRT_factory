function [ nearest, ed ] = find_nearest( node, In_list_ID, x)
% Find the nearest node from nodes in the tree (In_list_ID) to the node (x) and the minimum distance

%param = load('param.mat');
%allowable_angle = param.allowable_angle;
candidate_parent_node_ID = [];
    

    for ii = 1:length(In_list_ID)
        node_ID = In_list_ID(ii);
        
        time_at_nodejj   = node(node_ID).x(3);
        angle_difference = angle_difference_nodejj_and_x(node(node_ID),x);
    
        if time_at_nodejj < x(3) %&& abs(angle_difference) < allowable_angle
            candidate_parent_node_ID(end+1) =  node_ID;
        end
    
    end
    
    
    if isempty(candidate_parent_node_ID) % もし親ノードの候補が見つからなかったとき、
    
        ed      = 0; % ed<radius_minにしてサンプルをやり直す。
        nearest = 0;
    
    else % 親ノードの候補が存在するとき、各候補と新ノード間の距離を計算
        candidate_parent_node = reshape([node(candidate_parent_node_ID).x], [3, numel(candidate_parent_node_ID)]).';
        d = sqrt( sum( ((candidate_parent_node - x).^2)'));
    
        % 最小距離，最小となる既存のノードの番号
        [ed, d_min_index] = min(d);
        nearest = candidate_parent_node_ID(d_min_index);
    
    end

end

    



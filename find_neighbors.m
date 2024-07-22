function [ neighbor_ID, dist_xk2node_i, plotobj_neighbors] = find_neighbors( x, node, radius, In_list_ID_without_nearestID, Is_only_neibor_ID, is_rewiring)
% 新しくできたノードに対する近接ノードを探す．
% ※近接ノード： 新ノードからあらかじめ決めたradius以内にある点
%param = load('param.mat');
%allowable_angle = param.allowable_angle;

neighbor_ID = []; dist_xk2node_i = 0; plotobj_neighbors = plot3(0,0,0);

dim = numel(x);

candidate_neighbor_node_ID = [];
for ii = 1:length(In_list_ID_without_nearestID)
    node_ID = In_list_ID_without_nearestID(ii);
    
    time_at_nodejj   = node(node_ID).x(3);
    angle_difference = angle_difference_nodejj_and_x(node(node_ID),x);
    
    if is_rewiring == false
        if time_at_nodejj < x(3) %&& abs(angle_difference) < allowable_angle
            candidate_neighbor_node_ID(end+1) =  node_ID;
        end
    elseif is_rewiring == true
        if time_at_nodejj > x(3) %&& abs(angle_difference) < allowable_angle
            candidate_neighbor_node_ID(end+1) =  node_ID;
        end
    end
end

%%% 半径radiusの円柱を図示
[X,Y,Z] = cylinder(radius);
X = X+x(1); Y = Y+x(2); Z = Z*x(3);
plotobj_cylinder = surf(X,Y,Z,'FaceAlpha',0.3,'FaceColor',[0.6510 0.6510 0.6510],'EdgeColor',[0.6510 0.6510 0.6510]);

if ~isempty(candidate_neighbor_node_ID)

    candidate_neighbor_node = reshape([node(candidate_neighbor_node_ID).x], [3, numel(candidate_neighbor_node_ID)]).';
    d                       = sqrt( sum( ((candidate_neighbor_node - x).^2)'));

    Is_neighbor = d <= radius+0.001;
    
    % neighborノードを緑にプロット
    neighbor_ID = candidate_neighbor_node_ID( Is_neighbor );
    if ~isempty(neighbor_ID)
        for ii =1:length(neighbor_ID)
            plotobj_neighbors(ii) = plot3(node(neighbor_ID(ii)).x(1),node(neighbor_ID(ii)).x(2),node(neighbor_ID(ii)).x(3),'g.','MarkerSize',15);
        end 
    end

    if Is_only_neibor_ID
        dist_xk2node_i = [];
    else
%         dist_xk2node_i = [];
%         for ii =1:length(neighbor_ID)
%             dist_xk2node_i(end+1) = calc_cost(node(neighbor_ID(ii)), x, theta_from_nearest, omega_from_nearest, v_from_nearest);
%         end
%         
        %[dig, dInf, P_prime_list] = dist_ig_mat( x_nei_mat, P_nei_mat, x.', P, alpha, R);
        %dist_xk2node_i = dig( Is_neighbor ); % total cost
        % noissue_flagノードからノードiiまでの距離を格納した行ベクトル
        dist_xk2node_i = d( Is_neighbor );
        
        
    end

end

delete(plotobj_cylinder);


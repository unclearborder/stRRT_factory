function [x, nearest, plotobj_scale_newpoint,plotobj_ellipse_cylinder, theta, omega, v] = sample_x_P_randomly(node,In_list_ID, r_attempt, path, min_path_leng)

param = load('param.mat');
map = load('map.mat');
Dc            = param.Dc;
bound         = param.bound;

% 障害物と衝突せず，領域の境界からはみ出ない新しいノードを生成する．
num_obs_edge_ini = length(map.AreaMap.name); % number of edges in obstacles

while true
    
    % 新しいノード ｘ を円柱内部に生成．(障害物の内部に生成してしまったらやり直し)
    obstacle_issue = 1;
    while  obstacle_issue == 1

        [x, obstacle_issue, nearest, plotobj_scale_newpoint,plotobj_ellipse_cylinder ] = sample_polyshape_check(node,In_list_ID,r_attempt);
        
        % 新しく生成した点が棄却(障害物の内部に生成されてしまったとき)
        if obstacle_issue == 1
            delete(plotobj_scale_newpoint);
        end
        
    end
    
            
    % 障害物と船と一定距離(Dc)を保っているかチェック．
    % for k = 1:num_obs_edge_ini
    % %     dist_Cnt2ObsEdge = dist_point2lineseg(x(1:2), obstacle_edge(k).start, obstacle_edge(k).end);
    % % 
    % %     if dist_Cnt2ObsEdge < Dc
    % %         % 衝突発生，このforループを抜けてノードの生成からやり直し．
    % %         obstacle_issue = true;
    % %         break
    % %     end
    % end
    % 
    % 探索している領域の境界との衝突チェック
    % boundary_issue = boundary_check(x,Dc, bound);

    % ノンホロ制約を満たしてその点に到達できるかチェック
    % [nonholo_issue, theta, omega, v] = nonholonomic_check2(x, node(nearest));
   
    % 衝突なしかつノンホロOKなら終了
    if obstacle_issue == false
        break
    end
    
    % 新しく生成した点は棄却された
    delete(plotobj_scale_newpoint);
    delete(plotobj_ellipse_cylinder);


end
    theta = param.min_turning_radius(1);
    omega = param.omega_max(1);
    v = param.v_max(1);

end

% dist_point2lineseg(x, edge.start, edge.end) returns the distence
% between point x and the line segment between edge.start and edge.end
function dist_pt2LineSeg = dist_point2lineseg(pt, line_St, line_Ed)
    line_seg = line_Ed - line_St;
    lineSt2_pt = pt - line_St;
    
    cross_twoLine = dot(lineSt2_pt, line_seg);
    cross_lineSeg = dot(line_seg, line_seg);
    
    if cross_twoLine <= 0
        dist_pt2LineSeg = norm(lineSt2_pt);
    elseif cross_twoLine < cross_lineSeg
        d2 = cross_twoLine / norm(line_seg);
        dist_pt2LineSeg = sqrt( norm(lineSt2_pt)^2 - d2^2 );
    else
        lineEd2_pt = pt - line_Ed;
        dist_pt2LineSeg = norm(lineEd2_pt);
    end
end
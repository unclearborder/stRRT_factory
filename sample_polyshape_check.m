function [x, obstacle_issue, nearest, plotobj_scale_newpoint,plotobj_ellipse_cylinder] = sample_polyshape_check(node, In_list_ID, radius,path, min_path_leng)

param = load('param.mat');
map = load('map.mat');
radius_min    = param.radius_min;
t_max         = param.t_max;
Dc = param.Dc;

x = zeros(1,3);
ed = 0;

num_obs_edge_ini = length(map.AreaMap.name);
%%$ 新しいノードの生成
while ed < radius_min % 新しく生成したノードの位置が近すぎたらやり直す．

%     if isempty(path) % 経路が見つかっていないとき、円柱内部からサンプリング
%         x = Sample_in_cylinder(node(1).x,t_max);

%         x = Sample_in_area_randomly(node,In_list_ID);
        x = Sample_in_area_zero(node,In_list_ID);
        plotobj_newpoint = plot3(x(1),x(2),x(3),'k.','MarkerSize',15);
        plotobj_ellipse_cylinder = plot3(0,0,0);


%     else  % 経路が見つかっているとき、Informed subsetかつ円柱からサンプリング 
% 
%         cylinder_issue = 1;
%         while cylinder_issue == 1
%             
%             [x, plotobj_ellipse_cylinder] = Sample_in_informed_subset(node(1).x(1:2),node(path(1)).x(1:2),node(path(1)).x(3),min_path_leng);
% 
%             plotobj_newpoint = plot3(x(1),x(2),x(3),'k.','MarkerSize',15);
%             cylinder_issue = sample_cylinder_check(x,node(1).x(1:2));
% 
%             if cylinder_issue == 1
%                 delete(plotobj_newpoint);
%                 delete(plotobj_ellipse_cylinder);
%             end    
% 
%         end
%     end

    % 最近接ノードとそのノードまでの距離を求める． 
%     [nearest,ed] = find_nearest(node, In_list_ID, x); 
    [nearest,ed] = find_nearest_2Djudge(node, In_list_ID, x);
    % サンプリングをやり直すときはそのプロットを削除する
    if ed < radius_min
        delete(plotobj_newpoint);
        delete(plotobj_ellipse_cylinder);
    end
 
end
%%% 親ノードから新しくサンプリングした点まで一定距離だけ枝を伸ばしそこを新ノードとする．
[x, plotobj_scale_newpoint] = scale_rrt_point(ed,radius,x,node, nearest);

% 新しくサンプリングした点を消去
delete(plotobj_newpoint);

%%% 新ノードが障害物の内部に生成されていないかチェック
obstacle_issue_ii = false(num_obs_edge_ini);
for ii=1:num_obs_edge_ini
%    if (inpolygon(x(1), x(2), obs_polyshape(ii).x, obs_polyshape(ii).y )) 
%       obstacle_issue = true;
%       break;
%    end    
    if strcmp(map.AreaMap.type(ii),'circle') == true
       centerAxis = map.AreaMap.offset_center(ii,:);
       if norm(x(1:2)-centerAxis) <= map.AreaMap.r(ii) + param.Dc
           obstacle_issue_ii(ii) = true;
           
       end
    elseif ~strcmp(map.AreaMap.type(ii),'circle') == true
        if strcmp(map.AreaMap.type(ii),'door') == true
            obstacle_issue_ii(ii) = false;
        else
            obs_shape_c = (map.AreaMap.offset_start(ii,:)+map.AreaMap.offset_finish(ii,:))./2;
            obs_shape_l = [-map.AreaMap.x(ii)./2-Dc, map.AreaMap.y(ii)./2+Dc];
            obs_shape = [obs_shape_c+obs_shape_l;obs_shape_c-obs_shape_l];
            obs_polyshape_x = obs_shape([1 1 2 2],1);
            obs_polyshape_y = obs_shape([1 2 2 1],2);
            if inpolygon(x(1), x(2), obs_polyshape_x, obs_polyshape_y )
                obstacle_issue_ii(ii) = true;
                
            end
        end
    end
end
obstacle_issue = ~isempty(obstacle_issue_ii(obstacle_issue_ii == 1)); 
end

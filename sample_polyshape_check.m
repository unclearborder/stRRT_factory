function [x, obstacle_issue, nearest, plotobj_scale_newpoint,plotobj_ellipse_cylinder] = sample_polyshape_check(node, In_list_ID, radius)

param = load('param.mat');
map = load('map.mat');
radius_min    = param.radius_min;
t_max         = param.t_max;

x = zeros(1,3);
ed = 0;

num_obs_edge_ini = length(map.AreaMap.name);
%%$ 新しいノードの生成
while ed < radius_min % 新しく生成したノードの位置が近すぎたらやり直す．

%     if isempty(path) % 経路が見つかっていないとき、円柱内部からサンプリング
        x = Sample_in_cylinder(node(1).x,t_max);
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
    [nearest,ed] = find_nearest(node, In_list_ID, x); 
    
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
obstacle_issue = false;
for ii=1:num_obs_edge_ini
%    if (inpolygon(x(1), x(2), obs_polyshape(ii).x, obs_polyshape(ii).y )) 
%       obstacle_issue = true;
%       break;
%    end    
    if strcmp(map.AreaMap.type(ii),'circle') == true
       centerAxis = map.AreaMap.offset_center(ii);
       if norm(x(1:2)-centerAxis) <= map.AreaMap.r(ii)
           obstacle_issue = true;
           break;
       end
    elseif ~strcmp(map.AreaMap.type(ii),'door') == true
       obs_shape_x = [map.AreaMap.offset_start(ii,1),map.AreaMap.offset_finish(ii,1)];
       obs_shape_y = [map.AreaMap.offset_start(ii,2),map.AreaMap.offset_finish(ii,2)];
       obs_polyshape_x = obs_shape_x([1 1 2 2]);
       obs_polyshape_y = obs_shape_y([1 2 2 1]);
       if inpolygon(x(1), x(2), obs_polyshape_x, obs_polyshape_y )
           obstacle_issue = true;
           break;
       end
    end
end
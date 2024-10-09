clear all
close all
clc

I = 500; % MaxIteration
Dcs = [400 10 10];
t_max = 2400;
v_maxs = [300 10 10];
omega_maxs = [1/3*pi 0.1 0.01];
curvature_threshold_in_radian = pi/3;
allowable_angle = pi/2;
Probability_of_extracting_targetnode = 0.03;
min_turning_radiuss = [0 50 200];
w_t = 100; % コスト関数でノードiiからノードjjへの到達時間にかかる重み 
w_c = 0; % コスト関数でノードiiからノードjjへの曲率    にかかる重み
w_d = 100; % コスト関数でノードiiからノードjjへのｘｙ距離にかかる重み
w_a = 0; % コスト関数でノードiiからノードjjへの姿勢差  にかかる重み
w_arrival = 1000; % ゴール到着時刻に対する重み(find_optimal_path_2D内で使用)

% target setting
target_candidate = [1,7,18,33];
% Initial_target = [1,7,18,33];
Initial_target = [3,10,25,36];

In = length(Initial_target);
Tn = length(target_candidate);

Dc = Dcs(1);
v_max = v_maxs(1);
omega_max = omega_maxs(1);
min_turning_radius = min_turning_radiuss(1);

%% saving parameters
save_node_parent       = cell(I*In,1);
save_node_children     = cell(I*In,1);
save_node_children_num = cell(I*In,1);
save_node_value        = cell(I*In,1);
save_node_removed      = cell(I*In,1);
save_In_list_ID_i      = cell(I*In,1);
save_children_temp     = zeros(100*I*In,1);
save_path_candidate    = [];

% How often we save the path 
save_freq = 1;
saver(1:I/save_freq,1:In) = struct('path', [], 'node',[]);
min_path_data = zeros(I,In);
min_path = cell(I,In);


%% definition of the tree structure
ini_st      = -100;
ini_value   = 5000;

node(1:I*In) = struct('x',ini_st*ones(1,3), ...
                      'theta', ini_st, ...
                      'omega', ini_st, ...
                      'v'    , ini_st, ...
                      'parent', ini_st,...
                      'children',int16( ini_st*ones( I,1) ),...   % もともとは N → ceil(N/3)
                      'tree',0,...
                      'children_num', 0,...
                      'value', ini_value,... 
                      'removed', true);

%% Branch-and-bound parameters

%Every bnb_val step, the branch and bound delete the unnecessary nodes.
bnb_val = 10;
% To save the data
remove_ID = cell(I*In/bnb_val,1);


%% Variables used for the RRT

% neighboring (=rewiring) radius computation
radius = 1000; %neiborhood and rewirimg radius = 船が１ステップで進む距離
radius_min = inv(realmax); %minimum radius
% dimension of the space (Current version only works with "dim = 2")
dim=3; % dimension of the problem

%% Search area
load get_mapinfo/map_data
load get_mapinfo/Target_data

bound(1).x = [map_data{1}.map_offset(1),map_data{1}.map_offset(1)+map_limit(1)];
bound(2).x = [map_data{1}.map_offset(2),map_data{1}.map_offset(2)-map_limit(2)];
bound(3).x = [0,t_max];

%% The setting for initial node
for i = 1:In
    node(i).x           = [Target_data{1}.data.axis(Initial_target(i),:),0];
    node(i).theta       = i*pi/2;
    node(i).tree        = Initial_target(i);
    node(i).omega       = 0;
    node(i).v           = 1;
    node(i).parent      = 0;
    node(i).value       = 0;
    node(i).removed     = false;
    min_path_leng{i}    = -10^10; 
end


TargetsPoint    = Target_data{1}.data.axis(target_candidate(:),:);
TargetsArea     = reshape(TargetsPoint',1,2,4) + 1000*[cos([3*pi/4; pi/4]),sin([pi/4; -pi/4])];

save('param.mat','Dcs','Dc','t_max','v_maxs','v_max','omega_maxs','omega_max','curvature_threshold_in_radian','radius_min','In','Tn',...
     'bound','Initial_target','TargetsPoint','TargetsArea',"allowable_angle",'Probability_of_extracting_targetnode',...
     'target_candidate','min_turning_radiuss','min_turning_radius','w_t','w_c','w_d','w_a','w_arrival');

for r = 1:length(map_data)
    

    AreaMap = map_data{r}.data;
    save('map.mat','AreaMap');
    plot_environment_3D_multi
    
    SideIndex = [1 5 6 2;
                 4 8 7 3;
                 1 5 8 4;
                 2 6 7 3];
    obj_v_t = [obj_v_base; obj_v_tlim];
        for j = 1:length(SideIndex)
            patch('Vertices',obj_v_t(SideIndex(j,:),:),'Faces',obj_f,'FaceColor','m','FaceAlpha',0.6);
        end
    save_counter = 0;

    max_num_attempts = 0;
    path = cell(Tn,1);

    ii = In+1;
    while ii < I*In
        node(ii).removed = false;
        In_list_ID = find(~[node(1:ii-1).removed]).';
        num_list = numel(In_list_ID);

        r_attempt = radius;
        nbor_issue = 1;
        num_sampling = 0;

        while nbor_issue == 1
            [x, nearest, plotobj_scale_newpoint, theta_from_nearest, omega_from_nearest, v_from_nearest] = ...
            sample_x_P_randomly_noCylinder(node,In_list_ID, r_attempt,path, min_path_leng);

            disp('sampling x');
            num_sampling = num_sampling + 1;
            [parent, value, nbor_issue, plotobj_neighbors, theta, omega, v] = find_parent(x, node, r_attempt, In_list_ID, nearest, theta_from_nearest, omega_from_nearest, v_from_nearest);
            delete(plotobj_neighbors);
            if nbor_issue == 1
                delete(plotobj_scale_newpoint);
%                 delete(plotobj_ellipse_cylinder(ii));
            elseif nbor_issue == 0
                delete(plotobj_scale_newpoint);
                plot_robot(ii) = plot_robot_3D(x,Dc,'b');
                %text( x(1)+10,x(2)+10,x(3)+15,num2str(ii),'Color','w','FontSize',15)
                num_attempts(ii) = num_sampling;
                tree_num = node(parent).tree;
                tree_index = find(Initial_target == tree_num);
                % エッジをプロットする．エッジのオブジェクトをplotobj_edge(ノード番号)で保存する．
                plotobj_edge(ii) = plot3([x(1) node(parent).x(1)],[x(2) node(parent).x(2)],[x(3) node(parent).x(3)],'Color',EdgeColorSelect(tree_index),'LineWidth',2);
            end
            



        end
        node(ii).x      = x;
        node(ii).theta  = theta;
        node(ii).omega  = omega;
        node(ii).v      = v;
        node(ii).tree   = node(parent).tree;
        node(ii).value  = value;
        node(ii).parent = parent;
        node(parent).children_num = node(parent).children_num + 1;
        node(parent).children( node(parent).children_num ) = int16(ii);
        
        % path generate
        [path, min_path_leng] = find_optimal_path_2D_Multi( node, In_list_ID, ii, TargetsArea, TargetsPoint, I, Tn);
        min_path_data(ii) = min_path_leng; % length of the shortest path
        min_path{ii} = path; % shortest at step ii  
        % ゴールまでの経路があるならプロットする
        % プロット数が多いとき，毎回経路を表示すると時間が非常にかかるので，最後の１回だけ経路を表示する．
        if ~isempty(path)% && ii == N
%             delete(plotobj_ellipse_cylinder(1:ii-1));
            % 経路が更新されたとき or 経路が初めて見つかったとき
             if ~isequal(min_path{ii-1},min_path{ii}) %|| isempty(min_path{ii-1})
                newPath.candidate = min_path{ii};
                newPath.length = min_path_data(ii);
                newPath.tree = node(ii).tree;
                save_path_candidate = [save_path_candidate,newPath];
                plot_path_to_goal;
             end
    
        end

        if rem(ii,bnb_val) == 0
            if ~isempty(path) 
                [node, remove_ID{ii/bnb_val}] = branch_and_bound_2D(node,path,In_list_ID, ii, ini_st, dim); 
            end
        end

        disp(ii) % print the step
        disp(~isempty(path))   % show if a path is found

        if rem(ii,save_freq) == 0
            save_counter = save_counter + 1;
            saver(save_counter).path = path;
            saver(save_counter).node = ii;
        end
        
        % save the node list 
        In_list_ID_i = find(~[node(1:ii).removed]).';
        save_In_list_ID_i{ii} = In_list_ID_i;
        save_node_parent{ii} = [node([In_list_ID_i]).parent];
        
        % Save the necessary data
        cnt_save = 0;
        for kk = 1:numel(In_list_ID_i)
            children_for_save = node(In_list_ID_i(kk)).children;
            children_for_save(children_for_save == ini_st) = [];
            save_children_temp(cnt_save+1:cnt_save+numel(children_for_save),1) = children_for_save;
            cnt_save = cnt_save + numel(children_for_save);
        end
        
        save_children_temp(save_children_temp==0) = [];
        save_node_children{ii} = save_children_temp;
        save_node_children_num{ii} = [node(In_list_ID_i).children_num];
        save_node_value{ii} = [node(In_list_ID_i).value];
        save_node_removed{ii} = [node(In_list_ID_i).removed];
        
        drawnow
%         if movie_record_on
%             frame = getframe(gcf);
%             writeVideo(videoobj,frame);
%         end

        if x(3) > t_max
            break
        end
        ii = ii + 1;
    end



end

%     ii = Tn+1;
% 
%     while ii < I*Tn
% 
%     end

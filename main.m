clear all
close all
clc



I = 500; % MaxIteration
% Dc = [500 1000 2000];
Dc_m = [400 10 10];
t_max = 400;
v_max_m = [800 10 10];
omega_max_m = [pi 0.1 0.01];
curvature_threshold_in_radian = pi;     % 経路の最大許容角度差
allowable_angle = 3.14; % 新ノードが親ノードを探すときになす角がどの範囲内にあるものを対象とするか
Probability_of_extracting_targetnode = 0.03; % 適当な確率で目標点(target) を抽出
min_turning_radius_m = [50 50 200]; %最小回転半径
w_t = 1; % コスト関数でノードiiからノードjjへの到達時間にかかる重み 
w_c = 0; % コスト関数でノードiiからノードjjへの曲率    にかかる重み
w_d = 1; % コスト関数でノードiiからノードjjへのｘｙ距離にかかる重み
w_a = 0; % コスト関数でノードiiからノードjjへの姿勢差  にかかる重み
w_arrival = 0; % ゴール到着時刻に対する重み(find_optimal_path_2D内で使用)

Targetst = 7;
Targeted = 31;

m_r = 1;
Dc = Dc_m(m_r);
v_max = v_max_m(m_r);
omega_max = omega_max_m(m_r);
min_turning_radius = min_turning_radius_m(m_r);

movie_record_on = false;
rewiring_on = true;

%% 動画保存設定
speed = 100; 
Frate = 80; % 大きな数値にするほど、結果が高速で描画されていく
if movie_record_on
    videoobj = VideoWriter(append('results_2D_from',num2str(Targetst),'to',num2str(Targeted),'_withTime_randomly'));
    videoobj.FrameRate = Frate; % Framerate
    open(videoobj);
end

%% saving parameters
save_node_parent       = cell(I,1);
save_node_children     = cell(I,1);
save_node_children_num = cell(I,1);
save_node_value        = cell(I,1);
save_node_removed      = cell(I,1);
save_In_list_ID_i      = cell(I,1);
save_children_temp     = zeros(I*100,1);
save_path_candidate    = [];

% How often we save the path 
save_freq = 1;
saver(1:I/save_freq) = struct('path', [], 'node',[]);
min_path_data = zeros(I,1);
min_path = cell(I,1);



%%  Definition of the tree structure
% Just setting initial value 
ini_st    = -100; % initial value for unsampled nodes
ini_value = 5000; % initial cost for unsampled nodes

node(1:I) = struct('x',ini_st*ones(1,3), ...
                   'theta', ini_st, ...
                   'omega', ini_st, ...
                   'v'    , ini_st, ...
                   'parent', ini_st,...
                   'children',int16( ini_st*ones( I,1) ),...   % もともとは N → ceil(N/3)
                   'children_num', 0,...
                   'value', ini_value,... 
                   'removed', true);

%% Branch-and-bound parameters

%Every bnb_val step, the branch and bound delete the unnecessary nodes.
bnb_val = 10;
% To save the data
remove_ID = cell(I/bnb_val,1);

%% Variables used for the RRT*

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

SearchTarget = Target_data{1}.data.axis(Targeted,:);
target = SearchTarget+1000*[cos([3*pi/4; pi/4]),sin([pi/4; -pi/4])];

node(1).x        = [Target_data{1}.data.axis(Targetst,:),0]; % x,y,時間(初期位置)
node(1).theta    = pi/2;
% node(1).theta = pi/2 - acos(dot(SearchTarget-node(1).x(1:2),[0 1])/(norm(node(1).x(1:2)-SearchTarget)));% 初期姿勢
node(1).omega    = 0; 
node(1).v        = 1; 
node(1).parent   = 0;
node(1).value    = 0;
node(1).removed  = false;  % Node is in the tree and active
min_path_leng{1} = -10^10; % No path is found initially

save('param.mat','Dc','t_max','v_max','omega_max','curvature_threshold_in_radian','radius_min',...
     'bound','SearchTarget','target',"allowable_angle",'Probability_of_extracting_targetnode','min_turning_radius','w_t','w_c','w_d','w_a','w_arrival');



for r = 1:length(map_data)
    AreaMap = map_data{r}.data;
    save('map.mat','AreaMap');
    plot_environment_3D
    
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
    path = [];
    ii = 2;
    while isempty(path)
        node(ii).removed = false;
        In_list_ID = find(~[node(1:ii-1).removed]).';
        num_list = numel(In_list_ID);

        r_attempt = radius;
        nbor_issue = 1;
        num_sampling = 0;

        while nbor_issue == 1
%             [x, nearest, plotobj_scale_newpoint, plotobj_ellipse_cylinder(ii), theta_from_nearest, omega_from_nearest, v_from_nearest] = ...
%             sample_x_P_randomly(node,In_list_ID, r_attempt,path, min_path_leng);
            [x, nearest, plotobj_scale_newpoint, plotobj_ellipse_cylinder(ii)] = ...
            sample_x_P_randomly_noangle(node,In_list_ID, r_attempt,path, min_path_leng);

            disp('sampling x');
            num_sampling = num_sampling + 1;

%             [parent, value, nbor_issue, plotobj_neighbors, theta, omega, v] = find_parent(x, node, r_attempt, In_list_ID, nearest, theta_from_nearest, omega_from_nearest, v_from_nearest);
            [parent, value, nbor_issue, plotobj_neighbors] = find_parent_noangle(x, node, r_attempt, In_list_ID, nearest);
            delete(plotobj_neighbors);
            if nbor_issue == 1
                delete(plotobj_scale_newpoint);
                delete(plotobj_ellipse_cylinder(ii));
            elseif nbor_issue == 0
                delete(plotobj_scale_newpoint);
                plot_robot(ii) = plot_robot_3D(x,Dc,'b');
                %text( x(1)+10,x(2)+10,x(3)+15,num2str(ii),'Color','w','FontSize',15)
                num_attempts(ii) = num_sampling;
                
                % エッジをプロットする．エッジのオブジェクトをplotobj_edge(ノード番号)で保存する．
                plotobj_edge(ii) = plot3([x(1) node(parent).x(1)],[x(2) node(parent).x(2)],[x(3) node(parent).x(3)],'k','LineWidth',2);
            end
        end
        node(ii).x      = x;
%         node(ii).theta  = theta;
%         node(ii).omega  = omega;
%         node(ii).v      = v;
        node(ii).theta  = node(parent).theta;
        node(ii).omega  = node(parent).omega;
        node(ii).v      = node(parent).v;
        node(ii).value  = value;
        node(ii).parent = parent;
        node(parent).children_num = node(parent).children_num + 1;
        node(parent).children( node(parent).children_num ) = int16(ii);
        if rewiring_on       
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Rewiring %%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
     
            % rewiringを検討する近接ノードを探す．
            Is_only_neibor_ID = true; % 距離の計算を省略するためのフラグ
            is_rewiring       = true; % rewiringのときのfind_neighbors 
            
            % rewiringに使う新ノードを赤で表示する．
            p = plot3(x(1),x(2),x(3),'r.','MarkerSize',15);
            
            % rewiringを検討する近接ノードを緑で表示する  新ノード(赤)→既存ノード(緑)でコスト減少するならrewiringを行う。
            [nbors_rw_ID, ~, plotobj_neighbors_rewiring] = find_neighbors( x, node, r_attempt, In_list_ID, Is_only_neibor_ID, is_rewiring);
        
            % 衝突無しの近接ノードを探す． 
            num_rw_nbor = numel(nbors_rw_ID);
            issue_flag2 = true(num_rw_nbor, 1);
            
            for jj = 1:num_rw_nbor
                node_jj = node(nbors_rw_ID(jj)); 
                num_obs_edge_ini = length(map_data{r}.data.name);
                issue_flag1_j = false(num_obs_edge_ini,1);
                for j = 1:num_obs_edge_ini
                    if ~strcmp(map_data{r}.data.type(j),'circle') == true
                        if ~strcmp(map_data{r}.data.type(j),'door') == true
                            obstacle_edge = [map_data{r}.data.offset_start(j,:);
                                             map_data{r}.data.offset_finish(j,:)];
                            issue_flag1_j(j) = psuedo_obs_check_line_oct( x, node_jj, obstacle_edge, bound, Dc);
                        elseif strcmp(map_data{r}.data.type(j),'door') == true
                            issue_flag1_j(j) = false;
                        end
                            
                    elseif strcmp(map_data{r}.data.type(j),'circle') == true
                        obs_c = map_data{r}.data.offset_center(j);
                        obs_r = map_data{r}.data.r(j);
                        issue_flag1_j(j) =  psuedo_obs_check_circle_oct(x, node_jj, obs_c, obs_r, Dc);
                    end
                
                end
%                 issue_flag1_tmp = psuedo_obs_check_line_oct(    node(ii).x, node_jj, obstacle_edge, bound, Dc);
                issue_flag1_tmp = ~isempty(issue_flag1_j(issue_flag1_j == 1));
%                 issue_flag2_tmp = psuedo_obs_check_line_ship(   node(ii).x, node_jj, other_ship, Dc, v_o, t_max);
%                 issue_flag3_tmp = nonholonomic_check_rewiring(  node(ii).x, node_jj, node);
                issue_flag3_tmp = 1;
                issue_flag2(jj) = issue_flag1_tmp || issue_flag3_tmp;
            end
            
            nbors_rw_no_issue_ID = nbors_rw_ID(~issue_flag2);
            
            % 再配線すべきか検討するノードの表示を消去
            delete(plotobj_neighbors_rewiring);
        
            if ~isempty(nbors_rw_no_issue_ID)
                
                % reshaping the data
                x_nbors = [node(nbors_rw_no_issue_ID).x].';
                x_nbors_mat = reshape( x_nbors, [dim, numel(x_nbors)/dim]);
                
                % ユークリッド距離 from node ii to k
                dist_node_i2xk = dist_ig_mat(node(ii).x.', x_nbors_mat);
                
                % 再配線したときのvalueを計算
                val_new = node(ii).value + dist_node_i2xk.';
                
                % 再配線したときのvalueが既存の配線より小さくなるならrewiringを行う(rewired_better_ID)．
                rewired_ID_in_nbors_rw = find(val_new < [node(nbors_rw_no_issue_ID).value].');
                rewired_better_ID = nbors_rw_no_issue_ID(rewired_ID_in_nbors_rw);
                
                
                
                % rewiringを行うべきノードがある場合
                if ~isempty(rewired_better_ID)
                    
                    
                    for jj = 1:length(rewired_better_ID)
                        
                        % rewiringを行うべきノードを緑で表示
                        plotobj_node_rewired_better = plot3(node(rewired_better_ID(jj)).x(1),node(rewired_better_ID(jj)).x(2),node(rewired_better_ID(jj)).x(3),'g.','MarkerSize',15);
                    
                        % 古いエッジは消去
                        delete(plotobj_edge(rewired_better_ID(jj)));
                        
                        % 新しいエッジに書き換える．
                        plotobj_edge(rewired_better_ID(jj)) = plot3([x(1) node(rewired_better_ID(jj)).x(1)],[x(2) node(rewired_better_ID(jj)).x(2)],[x(3) node(rewired_better_ID(jj)).x(3)],'k','LineWidth',2);
                    
                        % rewiringを行うべきノードの表示を消去
                        delete(plotobj_node_rewired_better);
                        
                    end
                    
                    
                    
                    % rewiringを行うノードの現時点での親ノードのID
                    % その親ノードが持つ子ノードリストからrewiringされるノードを削除する必要がある．
                    old_parent_ID = [node(rewired_better_ID).parent].'; % old_parent: Vector size: same length with rewired_better_ID
        
                    % Matrix Size (2D): [length( node(1).children ) * length(rewired_better_ID)]
                    mat_parents_rewired_better = [node(old_parent_ID).children];
        
                    for k=1:length(rewired_better_ID) % rewigingを行うノードkを考える．
                        
                        % 新しい親ノードとvalueへと変更する．
                        % rewiringされるノードの新しい親は今回生成したノードiiへと変わる．
                        % rewiringされるノードへのvalueは今回生成したノードiiを経由した新しい経路のvalueへと変わる．
                        node(rewired_better_ID(k)).parent = ii;
                        node(rewired_better_ID(k)).value = val_new(rewired_ID_in_nbors_rw(k));
                        
                        % 古い親ノードの子ノードリストの "rewired_better_ID(k)" を "ini_st" に上書きする。
                        mat_parents_rewired_better([node(old_parent_ID(k)).children] == rewired_better_ID(k), k) = int16( ini_st );
                        node(old_parent_ID(k)).children = mat_parents_rewired_better(:, k);
                        
                        
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% rewiringされるノードの子ノード達の情報を更新する%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                    
                        % All direct children of the rewired node k
                        % We will update the information of these children in the following for loop     
                        % 先ほどrewiringを行ったノードkを親に持つ子ノードの親情報も全て変更しなければならない．
                        
                        direct_child_k_ID = [node(rewired_better_ID(k)).children].';
                        direct_child_k_ID( direct_child_k_ID == int16(ini_st) ) = [];
                        num_child = length(direct_child_k_ID); % number of direct children
                        
                        % rewiringされたノードkを含むすべての子ノードのID
                        descendants_k_ID = zeros(N,1);
                        descendants_k_ID(1:num_child) = direct_child_k_ID;
                        
                        count = num_child;
                        child = direct_child_k_ID;% list of children
                        
                
                % find all desecndant in the following loop
                while num_child > 0
                    
                    next_child_all = zeros(N,1);
                    cnt_child = 0;
                    
                    for cnt = 1:num_child
                        
                        next_child = [node( child(cnt) ).children].';
                        next_child(next_child == ini_st) = []; 
                        num_next_chi = length(next_child);

                        % Indirect children of node k is accumulated here
                        descendants_k_ID(count+1:count+num_next_chi) = next_child;
                        count = count + num_next_chi;

                        next_child_all(cnt_child+1:cnt_child+length(next_child)) = next_child;
                        cnt_child = cnt_child+length(next_child);
                    end
                    
                    next_child_all(next_child_all == 0) = [];
                    child = next_child_all;
                    num_child = length(child);   
                end
                
                descendants_k_ID(descendants_k_ID == 0) = [];
                num_descendants_k = length(descendants_k_ID); % list of all descendants
                
                % For all nodes which paths were changed, update their path length 
                % This updates do not contain "rewired_better_ID(k)", but all the nodes after "rewired_better_ID(k)".
                % This part also contains lossless refinement
                        for cnt=1:num_descendants_k
                            % From 'm' to 'n'.
                            n=descendants_k_ID(cnt);
                            m=node(n).parent;
                            xn=node(n).x;
                            xm=node(m).x;
           
                            % log in the new value 
                            dmn = dist_ig_mat(xm.',xn.');
                            node(n).value=node(m).value+dmn;         
                        end                    
                    end
                end  
            end
        delete(p);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Rewiring END %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        [path, min_path_leng] = find_optimal_path_2D( node, In_list_ID, ii, target, I);
        min_path_data(ii) = min_path_leng; % length of the shortest path
        min_path{ii} = path; % shortest at step ii  
        % ゴールまでの経路があるならプロットする
        % プロット数が多いとき，毎回経路を表示すると時間が非常にかかるので，最後の１回だけ経路を表示する．
        if ~isempty(path)% && ii == N
            delete(plotobj_ellipse_cylinder(1:ii-1));
            % 経路が更新されたとき or 経路が初めて見つかったとき
             if ~isequal(min_path{ii-1},min_path{ii}) %|| isempty(min_path{ii-1})
                newPath.candidate = min_path{ii};
                newPath.length = min_path_data(ii);
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
        if movie_record_on
            frame = getframe(gcf);
            writeVideo(videoobj,frame);
        end

        if x(3) > t_max
            break
        end
        ii = ii + 1;
    end


end
if movie_record_on
    close(videoobj);
end

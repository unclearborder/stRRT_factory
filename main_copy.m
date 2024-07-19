clear all
close all
clc

N     =  100; % Maximum number of nodes
Dc    = 750;      % 衝突回避半径
t_max = 2400;     % 予測する最大時間
v_max = 10;       % 自船の最高速度
omega_max = 0.01; % 自船の最高角速度
curvature_threshold_in_radian = pi/3;     % 経路の最大許容角度差
allowable_angle = pi/2; % 新ノードが親ノードを探すときになす角がどの範囲内にあるものを対象とするか
Probability_of_extracting_targetnode = 0.03; % 適当な確率で目標点(target) を抽出
min_turning_radius = 1000; %最小回転半径
w_t = 0; % コスト関数でノードiiからノードjjへの到達時間にかかる重み 
w_c = 0; % コスト関数でノードiiからノードjjへの曲率    にかかる重み
w_d = 1; % コスト関数でノードiiからノードjjへのｘｙ距離にかかる重み
w_a = 10000; % コスト関数でノードiiからノードjjへの姿勢差  にかかる重み
w_arrival = 0; % ゴール到着時刻に対する重み(find_optimal_path_2D内で使用)

environment_tokyoport = true; % true:東京湾、false:テスト環境
movie_record_on       = false; % true:動画保存
rewiring_on           = true; % true:rewiringを有効化 


%% 動画保存設定
speed = 100; 
Frate = 80; % 大きな数値にするほど、結果が高速で描画されていく
if movie_record_on
    videoobj = VideoWriter('results_2D.avi');
    videoobj.FrameRate = Frate; % Framerate
    open(videoobj);
end

%% Saving parameters
% Variables used for save data
save_node_parent       = cell(N,1);
save_node_children     = cell(N,1);
save_node_children_num = cell(N,1);
save_node_value        = cell(N,1);
save_node_removed      = cell(N,1);
save_In_list_ID_i      = cell(N,1);
save_children_temp     = zeros(N*100,1);

% How often we save the path 
save_freq = 1;
saver(1:N/save_freq) = struct('path', [], 'node',[]);
min_path_data = zeros(N,1);
min_path = cell(N,1);

%%  Definition of the tree structure
% Just setting initial value 
ini_st    = -100; % initial value for unsampled nodes
ini_value = 5000; % initial cost for unsampled nodes

node(1:N) = struct('x',ini_st*ones(1,3), ...
                   'theta', ini_st, ...
                   'omega', ini_st, ...
                   'v'    , ini_st, ...
                   'parent', ini_st,...
                   'children',int16( ini_st*ones( N,1) ),...   % もともとは N → ceil(N/3)
                   'children_num', 0,...
                   'value', ini_value,... 
                   'removed', true);

% Initilize node with the value which will not be outputted by algorithm
% node.x: The position (2-D) of the node and time  (1*3 vector)
% node.P: The covariance (2-D) of the node (2*2 matrix)
% node.parent: The ID of the parent node of each node 
%     (Since each node should have only one parent, this is a scalar value)
% node.children: The ID of the children nodes for each node   (vector)
%     (The size of node.children is determined under expectation "the maximum
%       number of children will not surpass the determined size here" )
% node.children_num: This expresses how many nodes has been assigned to 
%     the children of each node.   (scalar value)
%     ####### CAUTION ######## 
%     This value does not corresponds to the actual number of 
%     the children nodes, since the children is deleted in the rewired process.
%     The actual number is obtained by  
%     "A = node(parent_ID).children_num;  A(A==ini_st) = []; length( A )"
%     ########################
% node.value: The pathlength from the initial node to each node (scalar value)
% node.removed: If this value is true, the node is not under consideration.
%     For example, if branch and bound delete node i, node.removed  for node i
%     becomes true. (Boolean)
% The size of node.children is determined under expectation "the maximum
% number of children will not surpass the determined size here like N/3" 

%% Branch-and-bound parameters

%Every bnb_val step, the branch and bound delete the unnecessary nodes.
bnb_val = 10;
% To save the data
remove_ID = cell(N/bnb_val,1);

%% Variables used for the RRT*

% neighboring (=rewiring) radius computation
radius = 1000; %neiborhood and rewirimg radius = 船が１ステップで進む距離
radius_min = inv(realmax); %minimum radius
% dimension of the space (Current version only works with "dim = 2")
dim=3; % dimension of the problem


%% Enviroment definition and Properties
obstacle_edge = obstacle_multi();
obs_polyshape = obstacle_polyshape(); %definition of obstacles to use polyshape functionalities of Matlab

% Path planning area
load get_mapInfo/map_info

if environment_tokyoport
    
    % 探索領域
    bound(1).x = [0,x_upperbound]; % X軸
    bound(2).x = [0,y_upperbound]; % Y軸
    bound(3).x = [0, t_max];         % Z軸 (時間軸)

    % ゴール [xmin, xmax; ymin, ymax]
    target = [10000, 15000; 18000, 22000]; 
    
    % 他船 (等速直線運動)
    other_ship = [10000, 10000, -pi/2]; % x,y,theta
    other_ship(end+1,:) = [5000, 20000, 3*pi/2+atan(1/2)];
    other_ship(end+1,:) = [15000, 19000, pi+pi/4];
    v_o        = 3;
    v_o(end+1) = 3;
    v_o(end+1) = 3;

else
    
    bound(1).x = [5000,15000];
    bound(2).x = [0,10000];
    bound(3).x = [0, t_max]; 

    target = [9000, 11000; 8000, 10000];
    %target = [8000, 12000; 5000, 7000];

    other_ship          = [10000, 4500, -pi/2]; % x,y,theta
    other_ship(end+1,:) = [11000, 7000, -pi/2];
    other_ship(end+1,:) = [9000, 7000, -pi/2];
   
    v_o        = 0;
    v_o(end+1) = 0;
    v_o(end+1) = 0;


end

clearvars x_wall1 y_wall1 x_wall2 y_wall2 x_upperbound y_upperbound num_vertices_obs1 num_vertices_obs2
        
%% The setting for initial node
node(1).x        = [10000, 1000, 0]; % x,y,時間(初期位置)
node(1).theta    = pi/2;             % 初期姿勢
node(1).omega    = 0; 
node(1).v        = 1; 
node(1).parent   = 0;
node(1).value    = 0;
node(1).removed  = false;  % Node is in the tree and active
min_path_leng{1} = -10^10; % No path is found initially


% 環境 図示
plot_enviroment_3D;

% 初期点プロット
plot_ship_3D(node(1).x(1),node(1).x(2),node(1).theta, node(1).x(3),'b');

save('param.mat','Dc','t_max','v_max','omega_max','curvature_threshold_in_radian','radius_min','obstacle_edge',"obs_polyshape",...
     'bound','target','other_ship','v_o',"allowable_angle",'Probability_of_extracting_targetnode','min_turning_radius','w_t','w_c','w_d','w_a','w_arrival');

%% Main Algorithm

% Counter used for saving the data
save_counter = 0;
% 最大で何回ノードを生成しなおしたか(新しくサンプリングして、それが親ノードを見つけられなかったときの最大回数) 
max_num_attempts = 0;
path = [];
tic
% main loop of the IG RRT* Algorithm
for ii=2:N
    
    % New node ii becomes active and will be added to the list of nodes
    node(ii).removed = false;
    In_list_ID = find(~[node(1:ii-1).removed]).'; % A list contains IDs of active nodes 
    num_list = numel(In_list_ID); % compute the number of active nodes
    
    % Compute neighboring (=rewiring) radius 
    %dummy= 5*gamma_star*(log(num_list+1)/(num_list+1))^(1/dim);
    %r_attempt= min(radius,dummy);
    r_attempt = radius;
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%% 新しいノードをツリーへ挿入 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    
        nbor_issue = 1;
        num_sampling = 0;
      
        while nbor_issue == 1 % 既存のツリーに接続できる新しいノードが見つかるまで(=親ノードの存在する新ノードが生成されるまで)
            
            % フリースペースに新しいｘの生成と既存のツリー上で最近接のノードnearestの決定
            [x, nearest, plotobj_scale_newpoint, plotobj_ellipse_cylinder(ii), theta_from_nearest, omega_from_nearest, v_from_nearest] = ...
            sample_x_P_randomly(node,In_list_ID, r_attempt, path, min_path_leng);
            
            disp('sampled new x');
            num_sampling = num_sampling+1;

            % 親ノードの決定(親ノードを見つけらなかった場合，nbor_issue = 1)
            [parent, value, nbor_issue, plotobj_neighbors, theta, omega, v] = find_parent(x, node, r_attempt, In_list_ID, nearest, theta_from_nearest, omega_from_nearest, v_from_nearest);
        
            delete(plotobj_neighbors);
            
            if nbor_issue == 1   % 新しくサンプリングした点が棄却されたとき，その点のプロットを消去

                delete(plotobj_scale_newpoint);
                delete(plotobj_ellipse_cylinder(ii));

            elseif nbor_issue == 0 % 新しくサンプリングした点が採用されたとき，その点を青くプロット

                delete(plotobj_scale_newpoint);
                plot_ship_3D(x(1),x(2),theta,x(3),'b');
                %text( x(1)+10,x(2)+10,x(3)+15,num2str(ii),'Color','w','FontSize',15)
                num_attempts(ii) = num_sampling;
                
                % エッジをプロットする．エッジのオブジェクトをplotobj_edge(ノード番号)で保存する．
                plotobj_edge(ii) = plot3([x(1) node(parent).x(1)],[x(2) node(parent).x(2)],[x(3) node(parent).x(3)],'k','LineWidth',2);

            end
        end
        
        
        % 新しく決定したノード情報をnode(ii)に書き込む
        node(ii).x      = x;
        node(ii).theta  = theta;
        node(ii).omega  = omega;
        node(ii).v      = v;
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
        issue_flag1_tmp = psuedo_obs_check_line_oct(    node(ii).x, node_jj, obstacle_edge, bound, Dc);
        issue_flag2_tmp = psuedo_obs_check_line_ship(   node(ii).x, node_jj, other_ship, Dc, v_o, t_max);
        issue_flag3_tmp = nonholonomic_check_rewiring(  node(ii).x, node_jj, node);
        issue_flag2(jj) = issue_flag1_tmp || issue_flag2_tmp || issue_flag3_tmp;
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
    % Try to find the optimal path from the current set of nodes
    % If there is no path which reaches the target region, path = [].
    [path, min_path_leng] = find_optimal_path_2D( node, In_list_ID, ii, target, N);
    min_path_data(ii) = min_path_leng; % length of the shortest path
    min_path{ii} = path; % shortest at step ii  
    

    % ゴールまでの経路があるならプロットする
    % プロット数が多いとき，毎回経路を表示すると時間が非常にかかるので，最後の１回だけ経路を表示する．
    if ~isempty(path)% && ii == N
        delete(plotobj_ellipse_cylinder(1:ii-1));
        % 経路が更新されたとき or 経路が初めて見つかったとき
         if ~isequal(min_path{ii-1},min_path{ii}) %|| isempty(min_path{ii-1})
            plot_path_to_goal;
        end

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Branch-and-bound %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Run branch and bound every "bnb_val"
    if rem(ii,bnb_val) == 0
        if ~isempty(path) 
            [node, remove_ID{ii/bnb_val}] = branch_and_bound_2D(node,path,In_list_ID, ii, ini_st, dim); 
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % run-time output in command window  
    clc;
    disp(ii) % print the step
    disp(~isempty(path))   % show if a path is found
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % saving parameters
    % saving at save_freq frequency
    % a = 23;
    % b = 5;
    % rem(a,b) → 3 
    
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
end
if movie_record_on
    close(videoobj);
end
%% Output Data
%%%%%%%%%%%%%% All data should be saved here %%%%%%%%%%%%%%%
% The file name used for save the data
% Data is saved in "data" folder
% Name includes N, alpha value, safety percentage 
% savename = ['data/Multi_N',num2str(N),'_safety_', num2str(0.9)];
% savename(savename=='.') = [];
% save(savename)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
timeElapsed = toc;

%% シミュレーション情報のプロット
simu_info = {['Total Calc Time : ',num2str(timeElapsed),'s'];
             ['Number of Nodes : ', num2str(N)];
             ['Max Number of Attempts : ', num2str(max(num_attempts))]};

parameter_info ={['Collision Avoidance Radius : ',num2str(Dc),'m'];
                 ['Prediction Time : ',num2str(t_max),'s'];
                 ['Neiborhood and Rewirimg Radius  : ', num2str(radius), 'm'];
                 ['Max angle difference of path: ', num2str(rad2deg(curvature_threshold_in_radian)),'°'];
                 ['w_t  : ', num2str(w_t)];
                 ['w_c  : ', num2str(w_c)];
                 ['w_d  : ', num2str(w_d)];
                 ['w_arrival  : ', num2str(w_arrival)]};

if environment_tokyoport
    plotobj_info(1) = text(-5500,20000,3800,simu_info,'FontSize',15);
    plotobj_info(2) = text(2500,20000,3500,parameter_info,'FontSize',15);
else
    plotobj_info(1) = text(1600,10000,3600,simu_info,'FontSize',15);
    plotobj_info(2) = text(6000,10000,3200,parameter_info,'FontSize',15);
end

%% 各ノードを生成するのに何回サンプリングし直したかをプロット
% no_node = [1:N];
% figure
% hold on
% grid on
% fig_f = gcf;
% 
% % 家のPCでやるとき
% fig_f.Position = [103   517   722   461];
% % 学校のPCでやるとき
% fig_f.Position = [-1643 294 597 487];
% 
% axis([no_node(1) no_node(end) 0 max(num_attempts)+1])
% xlabel("Node Number")
% ylabel("Number of attempts")
% set(gca, 'FontName', 'Arial', 'FontSize', 15)
% 
% plot(no_node,num_attempts)
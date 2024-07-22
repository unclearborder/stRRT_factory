clear all
close all
clc

I = 1000; % MaxIteration
% Dc = [500 1000 2000];
Dc = [10 10 10];
t_max = 2400;
v_max = [10 10 10];
omega_max = [pi 0.1 0.01];
curvature_threshold_in_radian = pi/3;     % 経路の最大許容角度差
allowable_angle = pi/2; % 新ノードが親ノードを探すときになす角がどの範囲内にあるものを対象とするか
Probability_of_extracting_targetnode = 0.03; % 適当な確率で目標点(target) を抽出
min_turning_radius = [0 50 200]; %最小回転半径
w_t = 0; % コスト関数でノードiiからノードjjへの到達時間にかかる重み 
w_c = 0; % コスト関数でノードiiからノードjjへの曲率    にかかる重み
w_d = 1; % コスト関数でノードiiからノードjjへのｘｙ距離にかかる重み
w_a = 10000; % コスト関数でノードiiからノードjjへの姿勢差  にかかる重み
w_arrival = 0; % ゴール到着時刻に対する重み(find_optimal_path_2D内で使用)

%% saving parameters
save_node_parent       = cell(I,1);
save_node_children     = cell(I,1);
save_node_children_num = cell(I,1);
save_node_value        = cell(I,1);
save_node_removed      = cell(I,1);
save_In_list_ID_i      = cell(I,1);
save_children_temp     = zeros(I*100,1);

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
Targetst = 1;
Targeted = 39;

SearchTarget = Target_data{1}.data.axis(Targeted,:);

node(1).x        = [Target_data{1}.data.axis(Targetst,:),0]; % x,y,時間(初期位置)
node(1).theta    = pi/2;             % 初期姿勢
node(1).omega    = 0; 
node(1).v        = 1; 
node(1).parent   = 0;
node(1).value    = 0;
node(1).removed  = false;  % Node is in the tree and active
min_path_leng{1} = -10^10; % No path is found initially

save('param.mat','Dc','t_max','v_max','omega_max','curvature_threshold_in_radian','radius_min',...
     'bound','SearchTarget',"allowable_angle",'Probability_of_extracting_targetnode','min_turning_radius','w_t','w_c','w_d','w_a','w_arrival');



for r = 1:length(map_data)
    AreaMap = map_data{r}.data;
    save('map.mat','AreaMap');
    plot_environment_3D
    plot(Target_data{r}.data.axis(Targetst,1),Target_data{r}.data.axis(Targetst,2),'r.')
    
    save_counter = 0;

    max_num_attempts = 0;
    path = [];

    for ii = 2:I
        node(ii).removed = false;
        In_list_ID = find(~[node(1:ii-1).removed]).';
        num_list = numel(In_list_ID);

        r_attempt = radius;
        nbor_issue = 1;
        num_sampling = 0;

        while nbor_issue == 1
            [x, nearest, plotobj_scale_newpoint, plotobj_ellipse_cylinder(ii), theta_from_nearest, omega_from_nearest, v_from_nearest] = ...
            sample_x_P_randomly(node,In_list_ID, r_attempt);

            disp('sampling x');
            x
            num_sampling = num_sampling + 1;

            [parent, value, nbor_issue, plotobj_neighbors, theta, omega, v] = find_parent(x, node, r_attempt, In_list_ID, nearest, theta_from_nearest, omega_from_nearest, v_from_nearest);
            delete(plotobj_neighbors);
            if nbor_issue == 1
                delete(plotobj_scale_newpoint);
                delete(plotobj_ellipse_cylinder(ii));
            elseif nbor_issue == 0
                delete(plotobj_scale_newpoint);
%                 plot_ship_3D(x(1),x(2),theta,x(3),'b');
                %text( x(1)+10,x(2)+10,x(3)+15,num2str(ii),'Color','w','FontSize',15)
                num_attempts(ii) = num_sampling;
                
                % エッジをプロットする．エッジのオブジェクトをplotobj_edge(ノード番号)で保存する．
                plotobj_edge(ii) = plot3([x(1) node(parent).x(1)],[x(2) node(parent).x(2)],[x(3) node(parent).x(3)],'k','LineWidth',2);
            end
        end
        node(ii).x      = x;
        node(ii).theta  = theta;
        node(ii).omega  = omega;
        node(ii).v      = v;
        node(ii).value  = value;
        node(ii).parent = parent;
        node(parent).children_num = node(parent).children_num + 1;
        node(parent).children( node(parent).children_num ) = int16(ii);
    end
end

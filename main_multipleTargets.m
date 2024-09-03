clear all
close all
clc

I = 500; % MaxIteration
Dc = [400 10 10];
t_max = 2400;
v_max_m = [300 10 10];
omega_max_m = [1/3*pi 0.1 0.01];
curvature_threshold_in_radian = pi/3;
allowable_angle = pi/2;
Probability_of_extracting_targetnode = 0.03;
min_turning_radius_m = [0 50 200];
w_t = 100; % コスト関数でノードiiからノードjjへの到達時間にかかる重み 
w_c = 0; % コスト関数でノードiiからノードjjへの曲率    にかかる重み
w_d = 100; % コスト関数でノードiiからノードjjへのｘｙ距離にかかる重み
w_a = 0; % コスト関数でノードiiからノードjjへの姿勢差  にかかる重み
w_arrival = 1000; % ゴール到着時刻に対する重み(find_optimal_path_2D内で使用)

% target setting
target_candidate = [1,7,18,33];

Tn = length(target_candidate);


%% saving parameters
save_node_parent       = cell(I,Tn);
save_node_children     = cell(I,Tn);
save_node_children_num = cell(I,Tn);
save_node_value        = cell(I,Tn);
save_node_removed      = cell(I,Tn);
save_In_list_ID_i      = cell(I,Tn);
save_children_temp     = zeros(I*100,Tn);
save_path_candidate    = cell(1,Tn);

% How often we save the path 
save_freq = 1;
saver(1:I/save_freq,1:Tn) = struct('path', [], 'node',[]);
min_path_data = zeros(I,Tn);
min_path = cell(I,Tn);


%% definition of the tree structure
ini_st      = -100;
ini_value   = 5000;

node(1:I,1:Tn) = struct('x',ini_st*ones(1,3), ...
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

node(1).x        = [Target_data{1}.data.axis(Targetst,:),0]; % x,y,時間(初期位置)
% node(1).theta    = pi/4;
node(1).theta = pi/2 - acos(dot(SearchTarget-node(1).x(1:2),[0 1])/(norm(node(1).x(1:2)-SearchTarget)));% 初期姿勢
node(1).omega    = 0; 
node(1).v        = 1; 
node(1).parent   = 0;
node(1).value    = 0;
node(1).removed  = false;  % Node is in the tree and active
min_path_leng{1} = -10^10; % No path is found initially
clear all
close all
clc

I = 500; % MaxIteration
Dc = [400 10 10];
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

Tn = length(target_candidate);


%% saving parameters
save_node_parent       = cell(I*Tn,1);
save_node_children     = cell(I*Tn,1);
save_node_children_num = cell(I*Tn,1);
save_node_value        = cell(I*Tn,1);
save_node_removed      = cell(I*Tn,1);
save_In_list_ID_i      = cell(I*Tn,1);
save_children_temp     = zeros(100*I*Tn,1);
save_path_candidate    = cell(Tn,1);

% How often we save the path 
save_freq = 1;
saver(1:I/save_freq,1:Tn) = struct('path', [], 'node',[]);
min_path_data = zeros(I,Tn);
min_path = cell(I,Tn);


%% definition of the tree structure
ini_st      = -100;
ini_value   = 5000;

node(1:I*Tn) = struct('x',ini_st*ones(1,3), ...
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
remove_ID = cell(I/bnb_val,1);


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
for i = 1:Tn
    node(i).x           = [Target_data{1}.data.axis(target_candidate(i),:),0];
    node(i).theta       = i*pi/2;
    node(i).tree        = i;
    node(i).omega       = 0;
    node(i).v           = 1;
    node(i).parent      = 0;
    node(i).value       = 0;
    node(i).removed     = false;
    min_path_leng{i}    = -10^10; 
end


TargetsPoint    = Target_data{1}.data.axis(target_candidate(:),:);
TargetsArea     = reshape(TargetsPoint',1,2,4) + 1000*[cos([3*pi/4; pi/4]),sin([pi/4; -pi/4])];

save('param.mat','Dc','t_max','v_maxs','omega_maxs','curvature_threshold_in_radian','radius_min',...
     'bound','TargetsPoint','TargetsArea',"allowable_angle",'Probability_of_extracting_targetnode',...
     'min_turning_radiuss','w_t','w_c','w_d','w_a','w_arrival');

for r = 1:length(map_data)
    v_max = v_maxs(1);
    omega_max = omega_maxs(1);
    min_turning_radius = min_turning_radiuss(1);

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

    ii = Tn+1;
    while ii < I*Tn
        node(ii).removed = false;
        In_list_ID = find(~[node(1:ii-1).removed]).';
        num_list = numel(In_list_ID);

        r_attempt = radius;
        nbor_issue = 1;
        num_sampling = 0;

        while nbor_issue == 1
            [x, nearest, plotobj_scale_newpoint, theta_from_nearest, omega_from_nearest, v_from_nearest] = ...
            sample_x_P_randomly_noCylinder(node,In_list_ID, r_attempt,path, min_path_leng);




        end

    end



end

%     ii = Tn+1;
% 
%     while ii < I*Tn
% 
%     end

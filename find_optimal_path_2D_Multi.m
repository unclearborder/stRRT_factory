function [ path, min_path_leng ] = find_optimal_path_2D_Multi( node, In_list_ID, ii, target, Initial, N, Tn)
% finds the optimal path from the current set of nodes
% If there is no path which reaches the target region, path = [] and 
% min_path_leng = -10^10;
param = load('param.mat');
w_arrival  = param.w_arrival;

% ターゲット領域の範囲
x_min = reshape(target(1,1,:),1,Tn);
y_min = reshape(target(2,2,:),1,Tn);
x_max = reshape(target(2,1,:),1,Tn);
y_max = reshape(target(1,2,:),1,Tn);

% 
x_just = Initial(:,1)';
y_just = Initial(:,2)';

% 現在アクティブなノードのIDリスト
In_list_i_ID = [In_list_ID; ii];

x_all = [node(In_list_i_ID).x].';

% 時間成分の削除
x_all = reshape(x_all,3,[])';
x_all(:,3) = [];
x_all = reshape(x_all',[],1);

all_x = reshape( x_all, [2, numel(x_all)/2]).';

xmin_check = all_x(:,1) >= x_min; 
xmax_check = all_x(:,1) <= x_max;
ymin_check = all_x(:,2) >= y_min;
ymax_check = all_x(:,2) <= y_max;
xjust_check = all_x(:,1) ~= x_just;
yjust_check = all_x(:,2) ~= y_just;

% In_target_checkはnx1の配列，1になっている行がターゲット領域に入っているノード
In_target_check = xmin_check + xmax_check + ymin_check + ymax_check + xjust_check + yjust_check;

% 最短経路を見つける．

% ゴールに到達しているノードIDの検索
[In_list_ID_row, In_list_ID_col] = find(In_target_check == 6);
In_list_region_ID = In_list_i_ID(In_list_ID_row);
% 各ノードに向かう経路のvalueを持ってくる．
val_all = [node(In_list_region_ID).value].';

% ゴールにおける到達時間もコストとして考慮する．
allnode_ingoal = [node(In_list_region_ID).x];
arrival_time   = allnode_ingoal(3:3:end)';
val_all        = val_all+w_arrival.*arrival_time;


% 最小のvalueとなる経路が最短経路
[min_path_leng, min_ind] = min(val_all);
if isempty(min_path_leng)
    min_path_leng = -10^10;
end
ind = In_list_region_ID(min_ind); % the node in target region that has the minimum value

% ターゲット領域から初期ノードに向かって最短経路を構成するノードのリストからなるベクトルを作成する．
path = zeros(N,1);
count = 1;
if ind~=0
    path(count) = ind;
    m = node(ind).parent;
    while all(m~=1:Tn)
        count = count + 1;
        path(count) = m;
        ind = m;
        m = node(ind).parent;
    end
    path(count + 1) = 1;
end

path(path==0) = [];


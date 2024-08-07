function [node, remove_ID] = branch_and_bound_2D(node, path, In_list_ID, i, ini_st, dim)

% ゴールノードの位置
term_pos = node(path(1)).x;

In_list_i_ID = [In_list_ID; i];
pos_list = [node(In_list_i_ID).x].';
pos_list = reshape( pos_list, [dim, numel(pos_list)/dim]);

% 全てのノードに対して，上記のゴールノードまでの距離を計算する．
node_value_end = dist_ig_mat( pos_list, term_pos.');

% 現状で最適pathのvalueは...
path_len = node(path(1)).value;

% 各ノードがゴールノードと直通していると仮定した場合のvalueを計算する
min_len = node_value_end.' + [node( In_list_i_ID ).value].';

% もし上記の仮定で求めたvalueが現状の最適pathのvalueより大きい場合，そのpathを削除する．
remove_ID = In_list_i_ID(min_len > path_len);
remove_ID(remove_ID == path(1)) = [];


for k = 1:length(remove_ID)
    % The ID of node we will remove in this loop
    node(remove_ID(k)).removed = true;
    node(remove_ID(k)).children = ini_st;
end


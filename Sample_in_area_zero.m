function x = Sample_in_area_zero(node,In_list_ID)
  
param  = load('param.mat');
t_max = param.t_max;
% v_max  = param.v_max;
target = param.target; 
bound = param.bound;
P      = param.Probability_of_extracting_targetnode;
 
%%% 入力変数
% x_start：スタート位置(初期位置)([x; y; t]の1x3列ベクトル)
% t_max  ：考えている時間の最大値(z軸の最大値)
% 
%%% 出力変数
% x = [x,y,t]：円柱内にランダムサンプリングされた点(1x3)

% ゴール領域からサンプリングするかどうか
p = rand(1);
if p < P % ゴール領域から抽出するとき

    x = target(1,1) + (target(1,2)-target(1,1))*rand(1);
    y = target(2,1) + (target(2,2)-target(2,1))*rand(1);
    t = 0;
%     t = 0;
%     ij = randi([1 length(In_list_ID)],1,1);
%     t_bottom = node(In_list_ID(ij)).x(3);
%     t = t_bottom + (t_max-t_bottom)*rand(1);


else % 通常のサンプリング
    x = bound(1).x(1)+(bound(1).x(2)-bound(1).x(1))*rand(1);
    y = bound(2).x(1)+(bound(2).x(2)-bound(2).x(1))*rand(1);
%     t = t_max*rand(1);
    t = 0;
%     t = 0;
%     ij = randi([1 length(In_list_ID)],1,1);
%     t_bottom = node(In_list_ID(ij)).x(3);
%     t = t_bottom + (t_max-t_bottom)*rand(1);

end
    
    x = [x, y, t];

end
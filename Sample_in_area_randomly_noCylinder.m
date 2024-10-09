function [x, target_list] = Sample_in_area_randomly_noCylinder(Tn)
  
param  = load('param.mat');
t_max = param.t_max;
% v_max  = param.v_max;
target = param.TargetsArea;
candidate = param.target_candidate;
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
    r = randi([1,Tn],1,1);
    x = target(1,1,r) + (target(2,1,r)-target(1,1,r))*rand(1);
    y = target(1,2,r) + (target(2,2,r)-target(1,2,r))*rand(1);
    t = t_max*rand(1);
    target_list = candidate(r);
else % 通常のサンプリング
    x = bound(1).x(1)+(bound(1).x(2)-bound(1).x(1))*rand(1);
    y = bound(2).x(1)+(bound(2).x(2)-bound(2).x(1))*rand(1);
    t = t_max*rand(1);
    
    x_min = reshape(target(1,1,:),Tn,1);
    y_min = reshape(target(2,2,:),Tn,1);
    x_max = reshape(target(2,1,:),Tn,1);
    y_max = reshape(target(1,2,:),Tn,1);

    xmin_check = x >= x_min; 
    xmax_check = x <= x_max;
    ymin_check = y >= y_min;
    ymax_check = y <= y_max;

    In_target_check = xmin_check + xmax_check + ymin_check + ymax_check;
    In_target_ID = find(In_target_check == 4);

    if isempty(In_target_ID)
        target_list = 0;
    else
        target_list = In_target_ID;
    end

end
    
    x = [x, y, t];

end
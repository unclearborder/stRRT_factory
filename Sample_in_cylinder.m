function x = Sample_in_cylinder(x_start,t_max)
  
param  = load('param.mat');
v_max  = param.v_max;
target = param.target; 
P      = param.Probability_of_extracting_targetnode;
 
%%% 入力変数
% x_start：スタート位置(初期位置)([x; y; t]の1x3列ベクトル)
% t_max  ：考えている時間の最大値(z軸の最大値)
% 
%%% 出力変数
% x = [x,y,t]：円柱内にランダムサンプリングされた点(1x3)

% ゴール領域からサンプリングするかどうか
p = rand(1);
mu = (t_max+x_start(3))/2;
sigma = 1/4;
if p < P % ゴール領域から抽出するとき

    x = target(1,1) + (target(1,2)-target(1,1))*rand(1);
    y = target(2,1) + (target(2,2)-target(2,1))*rand(1);
%     x = target(1);
%     y = target(2);
    
    
    while true
        R = mu*chol(sigma);
        
        t = x_start(3)+(t_max-x_start(3))*rand;
%         t = -100;
        
%         while t <= x_start(3) || t_max <= t
%             z = randn*R;
%             t = x_start(3) + z;
%             
%         end
        r = v_max*t;

        if norm([x,y]-[x_start(1),x_start(2)]) < r
                break
        end
    
    end


else % 通常のサンプリング
    
    % 時間をランダムサンプリング
%     R = mu*chol(sigma);

    t = x_start(3)+(t_max-x_start(3))*rand;
%     t = -100;
%     
%     while t <= x_start(3) || t_max <= t
%         z = randn*R;
%         t = x_start(3) + z;
%         
%     end

    % その時間における円柱半径は，
    r = v_max*t;
        
    
    while true

%         x = (2*r*rand-r)+x_start(1);
%         %y = (2*r*rand-r)+x_start(2);
%         y = (r*rand)+x_start(2); % 初期位置の全面180°にサンプリングを集中

        phi = 2*pi*rand;
        r_rand = rand;
        x = r*r_rand*cos(phi)+x_start(1);
        y = r*r_rand*sin(phi)+x_start(2);   
            if norm([x,y]-[x_start(1),x_start(2)]) < r
                break
            end
    
    end

end
    
    x = [x, y, t];

end
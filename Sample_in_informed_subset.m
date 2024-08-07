function [x, plotobj_ellipse_cylinder] = Sample_in_informed_subset(x_start,x_goal,t_max,c_best)
% Informed RRT* で提案された楕円領域内部でのサンプリングを行う。
% https://arxiv.org/pdf/1404.2334.pdf
% 今回は、xy平面で楕円の楕円円柱を考える。

    %%% 入力変数
    % x_start：スタート位置(初期位置)([x; y]の1x2列ベクトル)
    % x_goal ：ゴール位置([x; y]の1x2列ベクトル)
    % t_max  ：現時点見つかっている経路の到達時間
    % c_best ：現時点で見つかっている経路のコスト
    %
    %%% 出力変数
    % x = [x,y,t]：楕円内にランダムサンプリングされた点(1x3)
    
    c_min    = norm(x_start-x_goal);
    x_center = (x_start+x_goal)/2;
    theta = atan2((x_goal(2)-x_start(2)),(x_goal(1)-x_start(1)));

    ra = c_best/2;
    rb = sqrt(c_best^2-c_min^2)/2;
    
    plotobj_ellipse_cylinder = plot_ellipse_cylinder(x_center,ra,rb,theta,t_max);

    psi = 2*pi*rand;
    a   = ra*rand;
    b   = rb*rand;
    x   = (a*cos(psi)*cos(theta)-b*sin(psi)*sin(theta))+x_center(1); 
    y   = (a*cos(psi)*sin(theta)+b*sin(psi)*cos(theta))+x_center(2);
    t  = t_max*rand;
    
    x = [x,y,t];

end





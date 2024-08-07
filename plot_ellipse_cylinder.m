function plotobj_ellipse_cylinder = plot_ellipse_cylinder(ellipse_center,ra,rb,theta,height_of_cylinder)
%%% 入力引数
% ra                ：x軸方向の長さ(スカラー)
% rb                ：y軸方向の長さ(スカラー)
% ellipse_center    ：楕円の中心([x, y]の1x2行ベクトル)
% height_of_cylinder：楕円柱の高さ(スカラー)
% theta             ： x軸から時計回りに測った楕円の傾き(rad)(スカラー)
%%%  
    u = linspace(0,2*pi) ;
    v = linspace(0,height_of_cylinder) ; 
    
    [U,V] = meshgrid(u,v) ;
    
    X = (ra*cos(U)*cos(theta)-rb*sin(U)*sin(theta))+ellipse_center(1) ; 
    Y = (ra*cos(U)*sin(theta)+rb*sin(U)*cos(theta))+ellipse_center(2) ;
    Z = V ;
    %surf(X,Y,Z,Z,'FaceAlpha',0.1,'FaceColor',[0.6510 0.6510 0.6510],'EdgeColor',[0.6510 0.6510 0.6510])
    plotobj_ellipse_cylinder = surf(X,Y,Z,'FaceAlpha',0.1,'FaceColor',[0.6510 0.6510 0.6510],'EdgeColor',[0.6510 0.6510 0.6510]);

end

function cylinder_issue = sample_cylinder_check(x,init_xy)

    param = load('param.mat');
    v_max = param.v_max;
    
    % 新たにサンプリングした点の時間は，
    time_now = x(3);
    
    % その時間における円柱の半径は，
    r = v_max*time_now;
    
    % 自船の初期位置を原点(init_xy = node(1).x)として，そこから新しくサンプリングした点までの距離は， (時間軸は無視，xy座標平面上で)
    d = norm(x(1:2)-init_xy);
    
    % 新しく生成したノードが円柱の内部に生成されていることをチェック
    cylinder_issue = 0;
    if d >= r
        cylinder_issue = 1; % 円柱外部にある場合，もう一度サンプリングしなおす．
    end

end

function cost = calc_cost(node_ii, node_jj, theta_jj, omega_jj, v_jj)

    % node(ii)→node(jj)間のコストを計算する関数
    % 入力：node(ii), node_jj,theta_jj,omega_jj,v_jj
    % 出力：コスト

    param = load('param.mat');
    w_t   = param.w_t;
    w_c   = param.w_c;
    w_d   = param.w_d;
    w_a   = param.w_a;

    x_ii     = node_ii.x(1:2);
    time_ii  = node_ii.x(3);
    theta_ii = mod(node_ii.theta,2*pi);
    omega_ii = node_ii.omega;
    v_ii     = node_ii.v;

    x_jj     = node_jj(1:2);
    time_jj  = node_jj(3);
    theta_jj = mod(theta_jj,2*pi);
    
%     fprintf('theta_ii: %s \n',num2str(rad2deg(theta_ii)));
%     fprintf('theta_jj: %s \n',num2str(rad2deg(theta_jj)));

    % コスト関数の各項
    time_difference  = time_ii-time_jj; 
    turning_radius_to_nodejj = v_jj/omega_jj;
    curvature                = 1/turning_radius_to_nodejj;
    distance_on_xy_plane     = norm(x_ii-x_jj);
    angle_difference         = abs(theta_ii-theta_jj);

    cost = w_t*(time_difference) + w_c*(curvature) + w_d*(distance_on_xy_plane) + w_a*(angle_difference);

end

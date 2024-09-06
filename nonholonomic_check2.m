function [issue_flag, theta_new, omega, v] = nonholonomic_check2(x, node_jj)
    % 新しい点と最近接点の２つのノード間でv,omega一定で動くとすると，v,omegaの値はどのくらいになるか計算する．
    % それが v_max, omega_maxを越えなければノンホロノミック拘束を満たす．
    % 参考：坂原，宮崎，枡谷："時空間RRTによる複数移動障害物を考慮したリアルタイム軌道生成" の(9),(10),(8)式

    param     = load('param.mat');
    v_max     = param.v_max;
    omega_max = param.omega_max;
    min_turning_radius = param.min_turning_radius;
    
    issue_flag = false;

    x_nbor     = node_jj.x(1:3);
    theta_nbor = node_jj.theta;
    
    x_new  = x(1:3);
    
    x1=x_nbor(1); y1=x_nbor(2); t1=x_nbor(3); theta1= theta_nbor;
    x2=x_new(1) ; y2=x_new(2) ; t2=x_new(3) ;
    
%%%%%% v,omega,theta2の計算
    direction_vec_to_x = [x2-x1;y2-y1];
    direction_vec_of_nodejj = [cos(theta1);sin(theta1)];
    delta_time = t2-t1;

    % z成分を０としておく．
    cross_vec = cross([direction_vec_to_x(1:2);0],[direction_vec_of_nodejj(1:2);0]);

    % ロボットの姿勢ベクトルがdirection_vec_to_xに対して左手側に存在しているか？
    on_the_left_side = false;
    if cross_vec(3) > 0 
        on_the_left_side = true;
    end


    if (direction_vec_to_x(1) > 0 && direction_vec_to_x(2) > 0) || (direction_vec_to_x(1) < 0 && direction_vec_to_x(2) > 0)% 目標点が第１象限 or 第２象限にあるとき

        if on_the_left_side == true 
            [v1, omega1, theta_goal1, v2, omega2, theta_goal2] = find_2paths_with_some_conditions(direction_vec_of_nodejj, direction_vec_to_x, delta_time);
        else
            % direction_vec_of_nodejjをdirection_vec_to_x に対して反転
            symmetry_direction_vec_of_nodejj = create_symmetric_vector_about_line(direction_vec_of_nodejj,direction_vec_to_x);
            % 経路を求める
            [v1, omega1, theta_goal1, v2, omega2, theta_goal2] = find_2paths_with_some_conditions(symmetry_direction_vec_of_nodejj, direction_vec_to_x, delta_time);

            % direction_vec_to_x に対して反転して元の経路を求める．
            symmetry_direction_vec_at_goal1 = [cos(theta_goal1); sin(theta_goal1)];
            direction_vec_at_goal1 = create_symmetric_vector_about_line(symmetry_direction_vec_at_goal1,direction_vec_to_x);
            omega1 = -omega1;
            theta_goal1 = atan2(direction_vec_at_goal1(2),direction_vec_at_goal1(1));

            symmetry_direction_vec_at_goal2 = [cos(theta_goal2); sin(theta_goal2)];   
            direction_vec_at_goal2 = create_symmetric_vector_about_line(symmetry_direction_vec_at_goal2,direction_vec_to_x);
            omega2 = -omega2;
            theta_goal2 = atan2(direction_vec_at_goal2(2),direction_vec_at_goal2(1));

        end

    elseif (direction_vec_to_x(1) < 0 && direction_vec_to_x(2) < 0) || (direction_vec_to_x(1) > 0 && direction_vec_to_x(2) < 0) % 目標点が第３象限 or 第４象限にあるとき

        if on_the_left_side == false 
            [v1, omega1, theta_goal1, v2, omega2, theta_goal2] = find_2paths_with_some_conditions(direction_vec_of_nodejj, direction_vec_to_x, delta_time);
        else
            % direction_vec_of_nodejjをdirection_vec_to_x に対して反転
            symmetry_direction_vec_of_nodejj = create_symmetric_vector_about_line(direction_vec_of_nodejj,direction_vec_to_x);
            % 経路を求める
            [v1, omega1, theta_goal1, v2, omega2, theta_goal2] = find_2paths_with_some_conditions(symmetry_direction_vec_of_nodejj, direction_vec_to_x, delta_time);

            % direction_vec_to_x に対して反転して元の経路を求める．
            symmetry_direction_vec_at_goal1 = [cos(theta_goal1); sin(theta_goal1)];
            direction_vec_at_goal1 = create_symmetric_vector_about_line(symmetry_direction_vec_at_goal1,direction_vec_to_x);
            omega1 = -omega1;
            theta_goal1 = atan2(direction_vec_at_goal1(2),direction_vec_at_goal1(1));

            symmetry_direction_vec_at_goal2 = [cos(theta_goal2); sin(theta_goal2)];   
            direction_vec_at_goal2 = create_symmetric_vector_about_line(symmetry_direction_vec_at_goal2,direction_vec_to_x);
            omega2 = -omega2;
            theta_goal2 = atan2(direction_vec_at_goal2(2),direction_vec_at_goal2(1));

        end


    elseif direction_vec_to_x(1) == 0  % 目標点がｘ軸上にあるとき

        theta_goal1 = -theta1+pi;
        omega = (theta_goal1-theta1)/(t2-t1);
        v     = omega*(y2-y1)/(-cos(theta_goal1)+cos(theta1));

    elseif direction_vec_to_x(2) == 0  % 目標点がｙ軸上にあるとき

        theta_goal1 = -theta1;
        omega = (theta_goal1-theta1)/(t2-t1);
        v     = omega*(x2-x1)/(sin(theta_goal1)-sin(theta1));

    end
%     isempty(v1)

    if     0 <= v1 && v1 <= v_max && abs(omega1) <= omega_max && abs(v1/omega1) >= min_turning_radius
        theta_new = theta_goal1;
        omega     = omega1;
        v         = v1;
    elseif 0 <= v2 && v2 <= v_max && abs(omega2) <= omega_max && abs(v2/omega2) >= min_turning_radius
        theta_new = theta_goal2;
        omega     = omega2;
        v         = v2;
    else
        issue_flag = 1; %  解なし
        theta_new = 0; omega = 0; v= 0;
    end

end
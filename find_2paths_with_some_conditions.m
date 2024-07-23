function [v1, omega1, theta_goal1, v2, omega2, theta_goal2] = find_2paths_with_some_conditions(direction_vec_of_nodejj, direction_vec_to_x, delta_time)

% with_some_conditions の意味とは．．．
% 「目標ノードが第１or ２ 象限に存在」 かつ 「direction_vec_to_xに対してロボットの姿勢ベクトルが左側に存在する」
% という２つの条件を満たすとき，前進，後退で目標地点まで到達する経路を求める．
    
    theta1 = mod(atan2(direction_vec_of_nodejj(2),direction_vec_of_nodejj(1)),2*pi); 
    a = direction_vec_to_x(1)/direction_vec_to_x(2);
    b = direction_vec_of_nodejj(2)+a*direction_vec_of_nodejj(1);
    
    alpha = atan2(a/sqrt(1+a^2),1/sqrt(1+a^2));
    
    % 「直進で目標点に向かう経路」と「バックして目標点に向かう経路」の２つを求める．
    theta_goal1 = mod(asin(b/sqrt(1+a^2))-alpha,2*pi);
    omega1 = (theta_goal1-theta1)/delta_time;
    v1     = omega1*direction_vec_to_x(1)/(sin(theta_goal1)-sin(theta1));
    
    theta_goal2 = mod(asin(b/sqrt(1+a^2))-alpha,2*pi);
    omega2      = (theta_goal2-theta1)/delta_time;
    
    if omega1 > 0 % theta_goal1-theta1>0
        theta_goal2 =  theta_goal1-2*pi;
        omega2 = (theta_goal2-theta1)/delta_time;
    elseif omega1 < 0 % theta_goal1-theta1<0
        theta_goal2 =  theta_goal1+2*pi;
        omega2 = (theta_goal2-theta1)/delta_time;
    end
 
    v2 = omega2*direction_vec_to_x(1)/(sin(theta_goal2)-sin(theta1));
    
%     fprintf('a: %s \n',num2str(a));
%     fprintf('b: %s \n',num2str(b));
%     fprintf('theta1: %s \n',num2str(rad2deg(theta1)));
%     fprintf('theta_goal1: %s \n',num2str(rad2deg(theta_goal1)));
%     fprintf('theta_goal2: %s \n',num2str(rad2deg(theta_goal2)));
%     fprintf('omega1: %s \n',num2str(omega1));
%     fprintf('omega2: %s \n',num2str(omega2));
    
end

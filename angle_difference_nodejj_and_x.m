function psi = angle_difference_nodejj_and_x(node_jj,x)
%%% 入力
% x       ：サンプリング点x
% node_ii ：親ノードの候補node_ii
%%% 出力
% psi：角度(ラジアン)

    theta = node_jj.theta;
    
    vec_nodejj      = [cos(theta); sin(theta)];
    vec_nodejj_to_x = [x(1)-node_jj.x(1);x(2)-node_jj.x(2)];
    
    cos_psi = ( dot(vec_nodejj,vec_nodejj_to_x) )/( norm(vec_nodejj)*norm(vec_nodejj_to_x)  );
    
    psi = acos(cos_psi);

end
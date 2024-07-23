function symmetry_direction_vec_of_nodejj = create_symmetric_vector_about_line(direction_vec_of_nodejj,direction_vec_to_x)

% 出力：direction_vec_of_nodejjをdirection_vec_to_xに対して反転したもの
% direction_vec_to_xからdirection_vec_of_nodejjまで測った２つのベクトルのなす角ksiは
ksi = atan2(direction_vec_of_nodejj(2),direction_vec_of_nodejj(1))-atan2(direction_vec_to_x(2),direction_vec_to_x(1));
%fprintf('ksi: %s \n',num2str(rad2deg(ksi)));

theta1 = atan2(direction_vec_of_nodejj(2),direction_vec_of_nodejj(1)); 
symmetry_direction_vec_of_nodejj = [cos(theta1-2*ksi);sin(theta1-2*ksi)];

end

